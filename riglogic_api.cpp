// riglogic_api.cpp
// Flat C API over OpenRigLogic for Emscripten/WASM.
// All geometry is stored in metres (CM_TO_M applied at load time).
// UVs are stored V-flipped (1-v) for WebGL convention.
// Face winding is CCW for Three.js front-face culling.

#include <emscripten.h>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include <riglogic/RigLogic.h>

static constexpr float CM_TO_M = 0.01f;
// Row-index-to-joint-index divisor for RBF pose joint output rows (Euler joint
// attribute layout: tx,ty,tz,rx,ry,rz,sx,sy,sz = 9 attributes per joint row).
static constexpr uint16_t ATTR_COUNT_PER_EULER_JOINT = 9;

// ─────────────────────────────────────────────────────────────────────────────
// Internal data structures
// ─────────────────────────────────────────────────────────────────────────────

struct GUIToRawMapping {
    std::vector<uint16_t> inputIndices;
    std::vector<uint16_t> outputIndices;
    std::vector<float>    fromValues;
    std::vector<float>    toValues;
    std::vector<float>    slopeValues;
    std::vector<float>    cutValues;
};

// Sparse blend shape — deltas stored in METRES (pre-scaled at load time).
struct BlendShapeTarget {
    uint16_t              channelIndex;
    std::vector<uint32_t> positionIndices;
    std::vector<float>    deltasX;   // metres
    std::vector<float>    deltasY;   // metres
    std::vector<float>    deltasZ;   // metres
};

// Fully pre-processed LOD0 mesh geometry, layout-vertex indexed.
// All positions and deltas are in METRES. UVs are V-flipped. Winding is CCW.
struct MeshGeometry {
    std::string  name;
    uint32_t     layoutCount   = 0;
    uint32_t     indexCount    = 0;
    uint16_t     maxInfluences = 0;

    std::vector<uint32_t> layoutPositionIndices;
    std::vector<float>    positions;  // metres, 3 * layoutCount
    std::vector<float>    uvs;        // V-flipped, 2 * layoutCount
    std::vector<float>    normals;    // 3 * layoutCount
    std::vector<uint32_t> indices;    // CCW winding
    std::vector<float>    skinWeights;
    std::vector<uint16_t> skinJointIndices;
    std::vector<BlendShapeTarget> blendShapes;
};

struct RLHandle {
    rl4::RigLogic*    rigLogic    = nullptr;
    rl4::RigInstance* rigInstance = nullptr;

    std::vector<std::string> jointNames;
    std::vector<std::string> guiControlNames;
    std::vector<std::string> rawControlNames;
    std::vector<std::string> blendShapeChannelNames;
    std::vector<std::string> animatedMapNames;
    std::vector<uint16_t>    jointParentIndices;

    // Neutral joint transforms from the DNA definition layer.
    // Translations are centimetres; rotations are Euler degrees in XYZ order.
    std::vector<float> neutralT;
    std::vector<float> neutralR;

    GUIToRawMapping           guiToRaw;
    std::vector<MeshGeometry> meshes;

    // Deduplicated union of joint indices written by RigLogic (RBF pose outputs,
    // twist outputs, swing outputs). Only these joints should be overwritten by
    // evaluate() output; all other joints are posed directly (driver bones) or
    // stay at their neutral/rest transform.
    std::vector<uint32_t> drivenJointIndices;
};

// ─────────────────────────────────────────────────────────────────────────────
// processMesh — called once per LOD0 mesh at create time
// ─────────────────────────────────────────────────────────────────────────────
static void processMesh(rl4::BinaryStreamReader* reader,
                        uint16_t mi,
                        MeshGeometry& geo)
{
    auto posXs = reader->getVertexPositionXs(mi);
    auto posYs = reader->getVertexPositionYs(mi);
    auto posZs = reader->getVertexPositionZs(mi);
    auto uvUs  = reader->getVertexTextureCoordinateUs(mi);
    auto uvVs  = reader->getVertexTextureCoordinateVs(mi);
    auto nrmXs = reader->getVertexNormalXs(mi);
    auto nrmYs = reader->getVertexNormalYs(mi);
    auto nrmZs = reader->getVertexNormalZs(mi);

    auto layoutPosIdx = reader->getVertexLayoutPositionIndices(mi);
    auto layoutUvIdx  = reader->getVertexLayoutTextureCoordinateIndices(mi);
    auto layoutNrmIdx = reader->getVertexLayoutNormalIndices(mi);

    const uint32_t layoutCount = reader->getVertexLayoutCount(mi);
    geo.layoutCount = layoutCount;
    geo.layoutPositionIndices.resize(layoutCount);
    geo.positions.resize(layoutCount * 3);
    geo.uvs.resize(layoutCount * 2);
    geo.normals.resize(layoutCount * 3);

    const bool hasUVs     = (uvUs.size() > 0);
    const bool hasNormals = (nrmXs.size() > 0);

    for (uint32_t i = 0; i < layoutCount; i++) {
        const uint32_t pi = layoutPosIdx[i];
        geo.layoutPositionIndices[i] = pi;

        // Scale cm → m at load time so JS receives metres directly
        geo.positions[i*3+0] = posXs[pi] * CM_TO_M;
        geo.positions[i*3+1] = posYs[pi] * CM_TO_M;
        geo.positions[i*3+2] = posZs[pi] * CM_TO_M;

        if (hasUVs) {
            const uint32_t ui = layoutUvIdx[i];
            geo.uvs[i*2+0] = uvUs[ui];
            geo.uvs[i*2+1] = 1.0f - uvVs[ui];  // V-flip: DNA=top-origin, WebGL=bottom-origin
        }

        if (hasNormals) {
            const uint32_t ni = layoutNrmIdx[i];
            geo.normals[i*3+0] = nrmXs[ni];
            geo.normals[i*3+1] = nrmYs[ni];
            geo.normals[i*3+2] = nrmZs[ni];
        }
    }

    // ── Face triangulation (fan) with CCW winding for Three.js ────────────
    // DNA/Maya face order is CW when viewed from outside; swap the last two
    // vertices of each fan triangle to produce CCW (Three.js front face).
    const uint32_t faceCount = reader->getFaceCount(mi);
    uint32_t totalTris = 0;
    for (uint32_t f = 0; f < faceCount; f++) {
        const auto fv = reader->getFaceVertexLayoutIndices(mi, f);
        if (fv.size() >= 3) totalTris += (uint32_t)(fv.size() - 2);
    }
    geo.indexCount = totalTris * 3;
    geo.indices.reserve(geo.indexCount);

    for (uint32_t f = 0; f < faceCount; f++) {
        const auto fv = reader->getFaceVertexLayoutIndices(mi, f);
        const uint32_t n = (uint32_t)fv.size();
        for (uint32_t t = 1; t + 1 < n; t++) {
            geo.indices.push_back(fv[0]);
            geo.indices.push_back(fv[t + 1]);  // swapped for CCW
            geo.indices.push_back(fv[t]);       // swapped for CCW
        }
    }

    // ── Skin weights ──────────────────────────────────────────────────────
    geo.maxInfluences = reader->getMaximumInfluencePerVertex(mi);
    if (geo.maxInfluences == 0) geo.maxInfluences = 4;
    geo.skinWeights.assign(layoutCount * geo.maxInfluences, 0.0f);
    geo.skinJointIndices.assign(layoutCount * geo.maxInfluences, 0);

    for (uint32_t i = 0; i < layoutCount; i++) {
        const uint32_t pi      = geo.layoutPositionIndices[i];
        const auto     weights = reader->getSkinWeightsValues(mi, pi);
        const auto     joints  = reader->getSkinWeightsJointIndices(mi, pi);
        const uint32_t wCount  = (uint32_t)std::min((size_t)geo.maxInfluences, weights.size());
        for (uint32_t w = 0; w < wCount; w++) {
            geo.skinWeights[i * geo.maxInfluences + w]      = weights[w];
            geo.skinJointIndices[i * geo.maxInfluences + w] = joints[w];
        }
    }

    // ── Blend shapes — deltas scaled cm → m at load time ─────────────────
    const uint16_t bsCount = reader->getBlendShapeTargetCount(mi);
    geo.blendShapes.resize(bsCount);

    for (uint16_t b = 0; b < bsCount; b++) {
        auto& bst        = geo.blendShapes[b];
        bst.channelIndex = reader->getBlendShapeChannelIndex(mi, b);

        const auto bsVertIdx = reader->getBlendShapeTargetVertexIndices(mi, b);
        const auto bsDeltaXs = reader->getBlendShapeTargetDeltaXs(mi, b);
        const auto bsDeltaYs = reader->getBlendShapeTargetDeltaYs(mi, b);
        const auto bsDeltaZs = reader->getBlendShapeTargetDeltaZs(mi, b);

        const uint32_t dCount = (uint32_t)bsVertIdx.size();
        bst.positionIndices.resize(dCount);
        bst.deltasX.resize(dCount);
        bst.deltasY.resize(dCount);
        bst.deltasZ.resize(dCount);

        for (uint32_t d = 0; d < dCount; d++) {
            bst.positionIndices[d] = bsVertIdx[d];
            // Scale cm → m so deltas match position scale
            bst.deltasX[d] = bsDeltaXs[d] * CM_TO_M;
            bst.deltasY[d] = bsDeltaYs[d] * CM_TO_M;
            bst.deltasZ[d] = bsDeltaZs[d] * CM_TO_M;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public C API
// ─────────────────────────────────────────────────────────────────────────────
extern "C" {

EMSCRIPTEN_KEEPALIVE
RLHandle* rl_create(const uint8_t* dnaData, uint32_t dnaSize) {
    const char* tmpPath = "/tmp/_rl_dna.dna";
    {
        FILE* f = std::fopen(tmpPath, "wb");
        if (!f) return nullptr;
        const std::size_t written = std::fwrite(dnaData, 1, dnaSize, f);
        std::fclose(f);
        if (written != dnaSize) return nullptr;
    }

    auto stream = rl4::makeScoped<rl4::FileStream>(
        tmpPath,
        rl4::FileStream::AccessMode::Read,
        rl4::FileStream::OpenMode::Binary
    );
    auto reader = rl4::makeScoped<rl4::BinaryStreamReader>(stream.get());
    reader->read();
    if (!rl4::Status::isOk()) { std::remove(tmpPath); return nullptr; }

    // Quaternion mode: 10 floats/joint [tx ty tz | qx qy qz qw | sx sy sz]
    rl4::Configuration config{};
    config.rotationType = rl4::RotationType::Quaternions;
    // Force scalar math instead of the default CalculationType::AnyVector.
    // AnyVector runtime-detects SSE/AVX/NEON support (via TwistSwingJointsBuilderFactory
    // and RBFBehaviorFactory) and picks a vectorized code path accordingly. Under
    // Emscripten/WASM that detection does not reliably reflect what actually got
    // compiled in, and the twist/swing joint solver (used by every *_correctiveRoot_*
    // joint) was silently producing NaN quaternion output as a result -- verified
    // against the real Blender addon (same DNA, same neutral pose, same RigLogic
    // core) producing correct near-identity output for the exact same joints, so
    // this is a WASM-specific SIMD-path bug, not an inherent library behavior.
    // Scalar is slower but this viewer evaluates a handful of times per frame, not
    // in a hot loop, so correctness wins outright over the vectorized speedup.
    config.calculationType = rl4::CalculationType::Scalar;
    // Configuration::floatingPointType defaults to HalfFloat. Half-float precision
    // on the near-zero rotations correctiveRoot joints carry at rest can round to
    // exact zero and trip a normalize()/divide-by-zero inside the twist/swing
    // solver, producing NaN quaternion output. Force full float precision.
    config.floatingPointType = rl4::FloatingPointType::Float;

    rl4::RigLogic* rl = rl4::RigLogic::create(reader.get(), config);
    if (!rl) { std::remove(tmpPath); return nullptr; }

    rl4::RigInstance* ri = rl4::RigInstance::create(rl);
    if (!ri) { rl4::RigLogic::destroy(rl); std::remove(tmpPath); return nullptr; }

    RLHandle* h    = new RLHandle();
    h->rigLogic    = rl;
    h->rigInstance = ri;
    ri->setLOD(0);

    // ── Definition layer ──────────────────────────────────────────────────
    {
        const uint16_t n = reader->getJointCount();
        h->jointNames.reserve(n);
        h->jointParentIndices.resize(n);
        for (uint16_t i = 0; i < n; i++) {
            auto sv = reader->getJointName(i);
            h->jointNames.emplace_back(sv.data(), sv.size());
            h->jointParentIndices[i] = reader->getJointParentIndex(i);
        }
    }
    // ── Neutral joint transforms: definition-layer bind pose ────────────────
    {
        const uint16_t n = reader->getJointCount();
        const auto tx = reader->getNeutralJointTranslationXs();
        const auto ty = reader->getNeutralJointTranslationYs();
        const auto tz = reader->getNeutralJointTranslationZs();
        const auto rx = reader->getNeutralJointRotationXs();
        const auto ry = reader->getNeutralJointRotationYs();
        const auto rz = reader->getNeutralJointRotationZs();

        h->neutralT.resize((size_t)n * 3);
        h->neutralR.resize((size_t)n * 3);
        for (uint16_t i = 0; i < n; i++) {
            h->neutralT[i*3+0] = tx[i];
            h->neutralT[i*3+1] = ty[i];
            h->neutralT[i*3+2] = tz[i];
            h->neutralR[i*3+0] = rx[i];
            h->neutralR[i*3+1] = ry[i];
            h->neutralR[i*3+2] = rz[i];
        }
    }

    {
        const uint16_t n = reader->getGUIControlCount();
        h->guiControlNames.reserve(n);
        for (uint16_t i = 0; i < n; i++) {
            auto sv = reader->getGUIControlName(i);
            h->guiControlNames.emplace_back(sv.data(), sv.size());
        }
    }
    {
        const uint16_t n = reader->getRawControlCount();
        h->rawControlNames.reserve(n);
        for (uint16_t i = 0; i < n; i++) {
            auto sv = reader->getRawControlName(i);
            h->rawControlNames.emplace_back(sv.data(), sv.size());
        }
    }
    {
        const uint16_t n = reader->getBlendShapeChannelCount();
        h->blendShapeChannelNames.reserve(n);
        for (uint16_t i = 0; i < n; i++) {
            auto sv = reader->getBlendShapeChannelName(i);
            h->blendShapeChannelNames.emplace_back(sv.data(), sv.size());
        }
    }
    {
        const uint16_t n = reader->getAnimatedMapCount();
        h->animatedMapNames.reserve(n);
        for (uint16_t i = 0; i < n; i++) {
            auto sv = reader->getAnimatedMapName(i);
            h->animatedMapNames.emplace_back(sv.data(), sv.size());
        }
    }

    // ── Behavior layer: GUI→raw mapping ──────────────────────────────────
    {
        const auto inputIdx  = reader->getGUIToRawInputIndices();
        const auto outputIdx = reader->getGUIToRawOutputIndices();
        const auto fromVals  = reader->getGUIToRawFromValues();
        const auto toVals    = reader->getGUIToRawToValues();
        const auto slopes    = reader->getGUIToRawSlopeValues();
        const auto cuts      = reader->getGUIToRawCutValues();
        const size_t n = inputIdx.size();
        h->guiToRaw.inputIndices.resize(n);
        h->guiToRaw.outputIndices.resize(n);
        h->guiToRaw.fromValues.resize(n);
        h->guiToRaw.toValues.resize(n);
        h->guiToRaw.slopeValues.resize(n);
        h->guiToRaw.cutValues.resize(n);
        for (size_t i = 0; i < n; i++) {
            h->guiToRaw.inputIndices[i]  = inputIdx[i];
            h->guiToRaw.outputIndices[i] = outputIdx[i];
            h->guiToRaw.fromValues[i]    = fromVals[i];
            h->guiToRaw.toValues[i]      = toVals[i];
            h->guiToRaw.slopeValues[i]   = slopes[i];
            h->guiToRaw.cutValues[i]     = cuts[i];
        }
    }

    // ── Driven-joint classification: RBF pose outputs + twist + swing outputs ──
    // Mirrors Blender's driven_bone_names / twist_bone_names / swing_bone_names:
    // these are the only joints RigLogic's evaluate() output should be applied to.
    {
        std::vector<uint32_t> driven;

        const uint16_t rbfSolverCount = reader->getRBFSolverCount();
        for (uint16_t s = 0; s < rbfSolverCount; s++) {
            const auto poseIndices = reader->getRBFSolverPoseIndices(s);
            for (uint16_t poseIndex : poseIndices) {
                const auto rowIndices = reader->getRBFPoseJointOutputIndices(poseIndex);
                for (uint16_t row : rowIndices) {
                    driven.push_back((uint32_t)(row / ATTR_COUNT_PER_EULER_JOINT));
                }
            }
        }

        const uint16_t twistCount = reader->getTwistCount();
        for (uint16_t t = 0; t < twistCount; t++) {
            const auto jointIndices = reader->getTwistOutputJointIndices(t);
            for (uint16_t ji : jointIndices) driven.push_back((uint32_t)ji);
        }

        const uint16_t swingCount = reader->getSwingCount();
        for (uint16_t s = 0; s < swingCount; s++) {
            const auto jointIndices = reader->getSwingOutputJointIndices(s);
            for (uint16_t ji : jointIndices) driven.push_back((uint32_t)ji);
        }

        std::sort(driven.begin(), driven.end());
        driven.erase(std::unique(driven.begin(), driven.end()), driven.end());
        h->drivenJointIndices = std::move(driven);
    }

    // ── Geometry layer: process all LOD0 meshes ───────────────────────────
    {
        const auto lod0Indices   = reader->getMeshIndicesForLOD(0);
        const uint16_t meshCount = (uint16_t)lod0Indices.size();
        h->meshes.resize(meshCount);
        for (uint16_t m = 0; m < meshCount; m++) {
            const uint16_t mi = lod0Indices[m];
            const auto sv = reader->getMeshName(mi);
            h->meshes[m].name = std::string(sv.data(), sv.size());
            processMesh(reader.get(), mi, h->meshes[m]);
        }
    }

    std::remove(tmpPath);
    return h;
}

// ── EVALUATE (GUI controls — primary interface) ───────────────────────────────
// outJoints: floats in centimetres (local, relative to parent joint).
// JS must still scale joint positions by 0.01 when setting bone.position.
//
// Split into two steps (rl_set_gui_controls, rl_calculate_and_get_outputs) so
// callers can inject driver-bone raw-control writes (rl_set_raw_control) in
// between. This matters because RigLogic::mapGUIToRawControls() -- called by
// rl_set_gui_controls -- zeroes the ENTIRE GUI-mapped raw-control range
// (ConditionalTable::calculateForward does `std::fill_n(outputs, outputCount,
// 0.0f)` before writing mapped entries), so any rl_set_raw_control call made
// before it would be silently wiped. Driver-bone raw controls must be set
// AFTER rl_set_gui_controls and BEFORE rl_calculate_and_get_outputs.
EMSCRIPTEN_KEEPALIVE
void rl_set_gui_controls(RLHandle* h, const float* guiControls, int guiCount) {
    if (!h) return;
    const int n = std::min(guiCount, (int)h->guiControlNames.size());
    for (int i = 0; i < n; i++) {
        h->rigInstance->setGUIControl((uint16_t)i, guiControls[i]);
    }
    h->rigLogic->mapGUIToRawControls(h->rigInstance);
}

EMSCRIPTEN_KEEPALIVE
void rl_calculate_and_get_outputs(
    RLHandle* h,
    float*    outJoints,   int jointFloatCount,
    float*    outBlends,   int blendCount,
    float*    outWrinkles, int wrinkleCount)
{
    if (!h) return;

    h->rigLogic->calculate(h->rigInstance);

    const auto joints = h->rigInstance->getJointOutputs();
    std::memcpy(outJoints, joints.data(),
                std::min((int)joints.size(), jointFloatCount) * sizeof(float));

    const auto blends = h->rigInstance->getBlendShapeOutputs();
    std::memcpy(outBlends, blends.data(),
                std::min((int)blends.size(), blendCount) * sizeof(float));

    const auto maps = h->rigInstance->getAnimatedMapOutputs();
    std::memcpy(outWrinkles, maps.data(),
                std::min((int)maps.size(), wrinkleCount) * sizeof(float));
}

// Convenience wrapper for pure facial evaluation (no driver-bone posing):
// rl_set_gui_controls + rl_calculate_and_get_outputs in one call.
EMSCRIPTEN_KEEPALIVE
void rl_evaluate_gui(
    RLHandle*    h,
    const float* guiControls, int guiCount,
    float*       outJoints,   int jointFloatCount,
    float*       outBlends,   int blendCount,
    float*       outWrinkles, int wrinkleCount)
{
    if (!h) return;
    rl_set_gui_controls(h, guiControls, guiCount);
    rl_calculate_and_get_outputs(h, outJoints, jointFloatCount, outBlends, blendCount, outWrinkles, wrinkleCount);
}

// ── EVALUATE (raw controls — low-level escape hatch) ──────────────────────────
EMSCRIPTEN_KEEPALIVE
void rl_evaluate(
    RLHandle*    h,
    const float* controls,  int controlCount,
    float*       outJoints, int jointFloatCount,
    float*       outBlends, int blendCount,
    float*       outWrinkles, int wrinkleCount)
{
    if (!h) return;

    rl4::ArrayView<float> rawCtrls = h->rigInstance->getRawControlValues();
    const int n = std::min(controlCount, (int)rawCtrls.size());
    for (int i = 0; i < n; i++) rawCtrls[i] = controls[i];

    h->rigLogic->calculate(h->rigInstance);

    const auto joints = h->rigInstance->getJointOutputs();
    std::memcpy(outJoints, joints.data(),
                std::min((int)joints.size(), jointFloatCount) * sizeof(float));

    const auto blends = h->rigInstance->getBlendShapeOutputs();
    std::memcpy(outBlends, blends.data(),
                std::min((int)blends.size(), blendCount) * sizeof(float));

    const auto maps = h->rigInstance->getAnimatedMapOutputs();
    std::memcpy(outWrinkles, maps.data(),
                std::min((int)maps.size(), wrinkleCount) * sizeof(float));
}

// ── POSING: raw-control writes + driven-joint classification ──────────────────
// Driver bones are posed directly by the caller (their Three.js bone quaternion
// IS the pose); this writes that quaternion component into RigLogic's raw
// control buffer so corrective (driven) joints can be solved from it.
EMSCRIPTEN_KEEPALIVE
void rl_set_raw_control(RLHandle* h, int index, float value) {
    if (!h || index < 0) return;
    rl4::ArrayView<float> rawCtrls = h->rigInstance->getRawControlValues();
    if ((size_t)index >= rawCtrls.size()) return;
    rawCtrls[index] = value;
}

EMSCRIPTEN_KEEPALIVE
int rl_get_driven_joint_count(RLHandle* h) {
    return h ? (int)h->drivenJointIndices.size() : 0;
}

EMSCRIPTEN_KEEPALIVE
void rl_get_driven_joint_indices(RLHandle* h, uint32_t* out, int count) {
    if (!h || !out) return;
    const auto& v = h->drivenJointIndices;
    std::memcpy(out, v.data(), (size_t)std::min(count, (int)v.size()) * sizeof(uint32_t));
}

EMSCRIPTEN_KEEPALIVE
void rl_get_neutral_joint_translations(RLHandle* h, float* out, int n) {
    if (!h || !out || n <= 0) return;
    const int count = std::min(n, (int)h->neutralT.size());
    std::memcpy(out, h->neutralT.data(), (size_t)count * sizeof(float));
}

EMSCRIPTEN_KEEPALIVE
void rl_get_neutral_joint_rotations(RLHandle* h, float* out, int n) {
    if (!h || !out || n <= 0) return;
    const int count = std::min(n, (int)h->neutralR.size());
    std::memcpy(out, h->neutralR.data(), (size_t)count * sizeof(float));
}

// GUI control ranges derived from the GUI→raw mapping definition.
// Controls without mapping entries fall back to [0, 1].
EMSCRIPTEN_KEEPALIVE
float rl_get_gui_control_min(RLHandle* h, int guiIndex) {
    if (!h || guiIndex < 0 || guiIndex >= (int)h->guiControlNames.size()) return 0.0f;
    float minValue = 0.0f;
    bool found = false;
    for (size_t i = 0; i < h->guiToRaw.inputIndices.size(); i++) {
        if ((int)h->guiToRaw.inputIndices[i] != guiIndex) continue;
        const float lo = std::min(h->guiToRaw.fromValues[i], h->guiToRaw.toValues[i]);
        if (!found || lo < minValue) minValue = lo;
        found = true;
    }
    return found ? minValue : 0.0f;
}

EMSCRIPTEN_KEEPALIVE
float rl_get_gui_control_max(RLHandle* h, int guiIndex) {
    if (!h || guiIndex < 0 || guiIndex >= (int)h->guiControlNames.size()) return 1.0f;
    float maxValue = 1.0f;
    bool found = false;
    for (size_t i = 0; i < h->guiToRaw.inputIndices.size(); i++) {
        if ((int)h->guiToRaw.inputIndices[i] != guiIndex) continue;
        const float hi = std::max(h->guiToRaw.fromValues[i], h->guiToRaw.toValues[i]);
        if (!found || hi > maxValue) maxValue = hi;
        found = true;
    }
    return found ? maxValue : 1.0f;
}

EMSCRIPTEN_KEEPALIVE
void rl_destroy(RLHandle* h) {
    if (!h) return;
    rl4::RigInstance::destroy(h->rigInstance);
    rl4::RigLogic::destroy(h->rigLogic);
    delete h;
}

EMSCRIPTEN_KEEPALIVE
void rl_set_lod(RLHandle* h, uint16_t lod) {
    if (h) h->rigInstance->setLOD(lod);
}

// ── BUFFER SIZE QUERIES ───────────────────────────────────────────────────────

EMSCRIPTEN_KEEPALIVE
int rl_get_raw_control_count(RLHandle* h) {
    return h ? (int)h->rigInstance->getRawControlCount() : 0;
}
EMSCRIPTEN_KEEPALIVE
int rl_get_gui_control_count(RLHandle* h) {
    return h ? (int)h->guiControlNames.size() : 0;
}
// Total float count for joint output buffer (quaternion mode = 10 × jointCount).
EMSCRIPTEN_KEEPALIVE
int rl_get_joint_output_float_count(RLHandle* h) {
    return h ? (int)h->rigInstance->getJointOutputs().size() : 0;
}
EMSCRIPTEN_KEEPALIVE
int rl_get_blend_count(RLHandle* h) {
    return h ? (int)h->rigInstance->getBlendShapeOutputs().size() : 0;
}
EMSCRIPTEN_KEEPALIVE
int rl_get_wrinkle_count(RLHandle* h) {
    return h ? (int)h->rigInstance->getAnimatedMapOutputs().size() : 0;
}

// ── COUNT QUERIES ─────────────────────────────────────────────────────────────

EMSCRIPTEN_KEEPALIVE
int rl_get_joint_count(RLHandle* h) {
    return h ? (int)h->jointNames.size() : 0;
}
EMSCRIPTEN_KEEPALIVE
int rl_get_blend_shape_channel_count(RLHandle* h) {
    return h ? (int)h->blendShapeChannelNames.size() : 0;
}
EMSCRIPTEN_KEEPALIVE
int rl_get_animated_map_count(RLHandle* h) {
    return h ? (int)h->animatedMapNames.size() : 0;
}
EMSCRIPTEN_KEEPALIVE
int rl_get_mesh_count(RLHandle* h) {
    return h ? (int)h->meshes.size() : 0;
}

// ── NAME LOOKUPS ──────────────────────────────────────────────────────────────

EMSCRIPTEN_KEEPALIVE
const char* rl_get_joint_name(RLHandle* h, int i) {
    if (!h || i < 0 || i >= (int)h->jointNames.size()) return "";
    return h->jointNames[i].c_str();
}
EMSCRIPTEN_KEEPALIVE
const char* rl_get_gui_control_name(RLHandle* h, int i) {
    if (!h || i < 0 || i >= (int)h->guiControlNames.size()) return "";
    return h->guiControlNames[i].c_str();
}
EMSCRIPTEN_KEEPALIVE
const char* rl_get_raw_control_name(RLHandle* h, int i) {
    if (!h || i < 0 || i >= (int)h->rawControlNames.size()) return "";
    return h->rawControlNames[i].c_str();
}
EMSCRIPTEN_KEEPALIVE
const char* rl_get_blend_shape_channel_name(RLHandle* h, int i) {
    if (!h || i < 0 || i >= (int)h->blendShapeChannelNames.size()) return "";
    return h->blendShapeChannelNames[i].c_str();
}
EMSCRIPTEN_KEEPALIVE
const char* rl_get_animated_map_name(RLHandle* h, int i) {
    if (!h || i < 0 || i >= (int)h->animatedMapNames.size()) return "";
    return h->animatedMapNames[i].c_str();
}
EMSCRIPTEN_KEEPALIVE
const char* rl_get_mesh_name(RLHandle* h, int i) {
    if (!h || i < 0 || i >= (int)h->meshes.size()) return "";
    return h->meshes[i].name.c_str();
}

// ── JOINT HIERARCHY ───────────────────────────────────────────────────────────
// Returns parent index, or -1 for root joint (parent==self) and invalid index.
EMSCRIPTEN_KEEPALIVE
int rl_get_joint_parent_index(RLHandle* h, int index) {
    if (!h || index < 0 || index >= (int)h->jointParentIndices.size()) return -1;
    const uint16_t p = h->jointParentIndices[index];
    if (p == 0xFFFF || (int)p == index) return -1;
    return (int)p;
}

// ── GEOMETRY: per-mesh metadata ───────────────────────────────────────────────

EMSCRIPTEN_KEEPALIVE
int rl_get_mesh_vertex_count(RLHandle* h, int mi) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size()) return 0;
    return (int)h->meshes[mi].layoutCount;
}
EMSCRIPTEN_KEEPALIVE
int rl_get_mesh_index_count(RLHandle* h, int mi) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size()) return 0;
    return (int)h->meshes[mi].indexCount;
}
EMSCRIPTEN_KEEPALIVE
int rl_get_mesh_max_influences(RLHandle* h, int mi) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size()) return 0;
    return (int)h->meshes[mi].maxInfluences;
}
EMSCRIPTEN_KEEPALIVE
int rl_get_mesh_blend_shape_count(RLHandle* h, int mi) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size()) return 0;
    return (int)h->meshes[mi].blendShapes.size();
}
EMSCRIPTEN_KEEPALIVE
int rl_get_mesh_blend_shape_channel_index(RLHandle* h, int mi, int bs) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size()) return -1;
    const auto& geo = h->meshes[mi];
    if (bs < 0 || bs >= (int)geo.blendShapes.size()) return -1;
    return (int)geo.blendShapes[bs].channelIndex;
}

// ── GEOMETRY: bulk data copy ──────────────────────────────────────────────────
// All data is pre-scaled to metres (positions, deltas) and V-flipped (UVs).
// JS receives metres directly — no further scaling needed for geometry.
// NOTE: Joint positions from rl_evaluate_gui are still in centimetres.

EMSCRIPTEN_KEEPALIVE
void rl_get_mesh_positions(RLHandle* h, int mi, float* out, int floatCount) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size() || !out) return;
    const auto& v = h->meshes[mi].positions;
    std::memcpy(out, v.data(), std::min(floatCount, (int)v.size()) * sizeof(float));
}
EMSCRIPTEN_KEEPALIVE
void rl_get_mesh_uvs(RLHandle* h, int mi, float* out, int floatCount) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size() || !out) return;
    const auto& v = h->meshes[mi].uvs;
    std::memcpy(out, v.data(), std::min(floatCount, (int)v.size()) * sizeof(float));
}
EMSCRIPTEN_KEEPALIVE
void rl_get_mesh_normals(RLHandle* h, int mi, float* out, int floatCount) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size() || !out) return;
    const auto& v = h->meshes[mi].normals;
    std::memcpy(out, v.data(), std::min(floatCount, (int)v.size()) * sizeof(float));
}
EMSCRIPTEN_KEEPALIVE
void rl_get_mesh_indices(RLHandle* h, int mi, uint32_t* out, int count) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size() || !out) return;
    const auto& v = h->meshes[mi].indices;
    std::memcpy(out, v.data(), std::min(count, (int)v.size()) * sizeof(uint32_t));
}
EMSCRIPTEN_KEEPALIVE
void rl_get_mesh_skin_weights(RLHandle* h, int mi, float* out, int floatCount) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size() || !out) return;
    const auto& v = h->meshes[mi].skinWeights;
    std::memcpy(out, v.data(), std::min(floatCount, (int)v.size()) * sizeof(float));
}
EMSCRIPTEN_KEEPALIVE
void rl_get_mesh_skin_joint_indices(RLHandle* h, int mi, uint16_t* out, int count) {
    if (!h || mi < 0 || mi >= (int)h->meshes.size() || !out) return;
    const auto& v = h->meshes[mi].skinJointIndices;
    std::memcpy(out, v.data(), std::min(count, (int)v.size()) * sizeof(uint16_t));
}

// Dense blend shape deltas in METRES (pre-scaled). Unaffected vertices = (0,0,0).
EMSCRIPTEN_KEEPALIVE
void rl_get_mesh_blend_shape_deltas(RLHandle* h, int mi, int bsIndex,
                                     float* out, int floatCount)
{
    if (!h || mi < 0 || mi >= (int)h->meshes.size() || !out) return;
    const auto& geo = h->meshes[mi];
    if (bsIndex < 0 || bsIndex >= (int)geo.blendShapes.size()) return;

    const auto&    bst    = geo.blendShapes[bsIndex];
    const uint32_t vCount = geo.layoutCount;
    const int      fill   = std::min(floatCount, (int)(vCount * 3));

    std::memset(out, 0, fill * sizeof(float));
    if (bst.positionIndices.empty()) return;

    std::unordered_map<uint32_t, uint32_t> posToD;
    posToD.reserve(bst.positionIndices.size());
    for (uint32_t d = 0; d < (uint32_t)bst.positionIndices.size(); d++)
        posToD[bst.positionIndices[d]] = d;

    for (uint32_t i = 0; i < vCount; i++) {
        const uint32_t posIdx = geo.layoutPositionIndices[i];
        auto it = posToD.find(posIdx);
        if (it != posToD.end()) {
            const uint32_t d  = it->second;
            const int      bi = (int)(i * 3);
            if (bi + 2 < fill) {
                out[bi + 0] = bst.deltasX[d];
                out[bi + 1] = bst.deltasY[d];
                out[bi + 2] = bst.deltasZ[d];
            }
        }
    }
}

} // extern "C"