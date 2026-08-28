# MetaHuman Web Viewer — a WebAssembly fork of OpenRigLogic

This is a fork of Epic Games' [OpenRigLogic](https://github.com/EpicGames/OpenRigLogic) that adds a WebAssembly build of RigLogic and a browser-based MetaHuman viewer built on Three.js. It lets a MetaHuman character — face, body, clothing, hair and eyebrows — be loaded, posed, and puppeted entirely client-side, with no engine or native runtime involved.

Two things were added on top of the upstream library:

1. **`riglogic_api.cpp`** — a WebAssembly bridge that exposes RigLogic/DNA evaluation to JavaScript.
2. **`index.html` / `js/` / `metahuman/`** — a Three.js demo that consumes that bridge to render and control a MetaHuman in the browser. These live at the repository root (rather than a `web/` subfolder) specifically so the repo can be served as-is by GitHub Pages — see [Deploying to GitHub Pages](#deploying-to-github-pages) below.

Everything below "Original OpenRigLogic README" is unmodified upstream documentation.

## 1. `riglogic_api.cpp` — the WebAssembly bridge

OpenRigLogic ships as a native C++ library with SWIG-generated Python bindings; neither targets a browser. `riglogic_api.cpp` is a flat, C-style API compiled with Emscripten that exposes just enough of RigLogic and the DNA reader to drive a character from JavaScript, using only primitive types and raw memory buffers (`malloc`/`memcpy`) at the boundary — the calling convention WebAssembly actually supports, since JS cannot call into arbitrary C++ objects or templates directly.

It's a single self-contained `.cpp` file (no bindings generator) that covers:

- **Lifecycle**: `rl_create` / `rl_destroy` load a `.dna` file (head or body) from an in-memory buffer and construct a `RigLogic` + `RigInstance` pair.
- **Evaluation**: `rl_set_gui_controls` + `rl_calculate_and_get_outputs` (split so raw-control writes — driver-bone poses, ARKit-driven blendshapes — can be injected between the GUI→raw mapping step and the final calculate), plus `rl_evaluate`/`rl_evaluate_gui` convenience wrappers, and `rl_set_raw_control` for direct raw-control writes.
- **Rig introspection**: joint hierarchy and names, GUI/raw control names and ranges, driven-joint classification (which joints are corrective vs. directly posable), all needed to build the control UI and skeleton without hardcoding anything DNA-specific.
- **Geometry extraction**: per-mesh positions, UVs (V-flipped for WebGL), normals, skin weights/joint indices, and sparse blend-shape deltas — everything needed to build a `THREE.SkinnedMesh` with morph targets, since the browser has no DNA reader of its own.

All geometry is converted to metres and CCW winding at load time so the output can be consumed by Three.js with no further conversion.

Build with Emscripten (`emcmake`/`emmake`) using the existing CMake target:

```shell
emcmake cmake -B build-web
emmake cmake --build build-web --target riglogic_wasm
```

This produces `riglogic_wasm.js` / `riglogic_wasm.wasm`, which are copied into `js/wasm/`.

## 2. The Three.js demo

```
index.html                # entire app: WASM loading, rig/mesh construction, UI, render loop
js/
├── wasm/                  # compiled riglogic_wasm.js / .wasm output
├── faceboard_layout.json  # face control board layout (see below) — hand-editable
└── arkit_mapping.json     # ARKit-52 → MetaHuman raw-control mapping (see below) — hand-editable
metahuman/<character>/     # per-character assets, e.g. metahuman/bazeel/
├── head.dna, body.dna
├── ExportManifest.json, Maps/, Masks/   # textures, wrinkle masks
├── clothes/                # skinned garment .glb + textures
└── groom/                  # hair/eyebrow .glb + textures
```

`index.html` is intentionally a single file: it loads the WASM module, builds the head and body skeletons and skinned meshes straight from DNA data, evaluates RigLogic every frame, and renders with Three.js. No build step or bundler is required to run it — serve the repository root with any static file server (all asset paths in `index.html` are relative, so this also works unmodified from a GitHub Pages subpath).

Two controls are included for posing the face:

- **Face control board** — a 2D draggable-dot UI reproducing the real MetaHuman face board from Blender's `character_dna` addon, extracted control-by-control (position, drag range, grouping boxes and labels) rather than hand-placed. Its layout lives entirely in `js/faceboard_layout.json`, so it can be hand-edited or regenerated without touching code.
- **ARKit-52 blendshape panel** — the same face driven by the standard 52 ARKit blendshape names instead of MetaHuman's native controls, for compatibility with ARKit-based facial capture pipelines. Moving an ARKit slider also updates the corresponding face-board dot live, so both stay visibly consistent when mixed. The mapping (`js/arkit_mapping.json`) is real data extracted from Epic's own `PA_MetaHuman_ARKit_Mapping` pose asset via [Dylanyz/ARKitRemap](https://github.com/Dylanyz/ARKitRemap) — credit to that project for surfacing it.

## 3. Using your own MetaHuman

The demo ships with one sample character (`metahuman/bazeel/`). To use your own:

1. **Face and body** — in Unreal Engine, open your MetaHuman and run **Export > DCC Export**. This produces `head.dna`, `body.dna`, textures, and `ExportManifest.json`. Copy these into a new `metahuman/<name>/` folder, matching the existing structure.
2. **Clothing and hair/eyebrows** — DCC Export does not include these, so they're exported separately from the Unreal project:
   - Export the character's outfit as a Skeletal Mesh FBX, and the hair/eyebrow Groom's card mesh (Static Mesh FBX) plus their base color textures.
   - Convert each FBX to `.glb` (this repo used Blender: import the FBX, then export glTF/GLB — see the comments in `index.html`'s `attachGarment`/`attachGroomSkinned` for the exact scale/transform pitfalls to avoid).
   - Place the resulting `.glb` + textures into `clothes/` and `groom/` under your character's folder.
3. **Wire it up** — update the character/mesh/texture filenames referenced near the top of `index.html` (`CHARACTER_DIR` and the `attachGarment`/`attachGroom` calls) to match your files.
4. **Face board / ARKit data** — `js/faceboard_layout.json` and `js/arkit_mapping.json` are keyed by this DNA's control names. If your MetaHuman uses the standard MetaHuman rig topology, they can be reused as-is; only regenerate them if your control names actually differ.

---

# OpenRigLogic

OpenRigLogic contains the RigLogic and DNA libraries that enable you to load a MetaHuman character with the same runtime rig evaluation as Unreal Engine. Both are available as native C++ libraries with Python bindings, ready to integrate into third-party content creation tools. OpenRigLogic is maintained by Epic Games.

MetaHuman has been adopted by some of the most successful games in the world, and is quickly becoming a standard for digital characters.  The DNA and RigLogic libraries found in OpenRigLogic are released in support of this developing standard.  The [RigLogic whitepaper](https://cdn2.unrealengine.com/rig-logic-whitepaper-v2-5c9f23f7e210.pdf) describes the design, file format, and runtime evaluation strategy in more detail.

## Contents

The OpenRigLogic repository contains:

* **DNA Library**: A C++ library for reading and writing MetaHuman DNA files, with Python bindings.  
* **RigLogic**: A highly optimized C++ library providing realtime evaluation of the MetaHuman rig, with Python bindings.  
* **Documentation:** A developer’s guide to integrating the RigLogic and DNA Libraries into your platform.

Sample assets and the MetaHuman Faceboard Control Rig may be acquired separately via the [OpenRigLogic Sample Content](https://www.fab.com/listings/27a81942-69bf-498e-a41f-004d0d2db37b) listing on Fab (available under the Fab Standard License).

## MetaHuman ecosystem compatibility

The implementation of technology such as OpenRigLogic creates an ecosystem of complimentary tools and products that enable a MetaHuman character to move seamlessly between different applications. 

Existing MetaHuman tools released by Epic Games (such as [MetaHuman Creator](https://dev.epicgames.com/documentation/metahuman/metahuman-creator) and [MetaHuman Animator](https://dev.epicgames.com/documentation/metahuman/metahuman-animator) in Unreal Engine and the [MetaHuman for Maya](https://www.fab.com/listings/9e3bf55e-d4c3-44fc-a3d4-ec4cb772ec29) and [MetaHuman for Houdini](https://www.fab.com/listings/7bbdfbb5-5eaf-4aa6-b32b-b8b048ebea25) plugins) are already part of this ecosystem. Your integration of OpenRigLogic into a third-party application enables you to take part in this ecosystem, providing compatibility with any MetaHuman character and tools you use to create them.

Typically, MetaHuman characters are created using [MetaHuman Creator](https://dev.epicgames.com/documentation/metahuman/metahuman-creator) in Unreal Engine. They can be exported using the **Export > DCC Export** tool. The exported package contains a DNA file for the head and a DNA file for the body-the inputs needed to drive the character with the libraries in OpenRigLogic. The character’s textures are also included.  Groom and clothing information is not currently part of this package.

## Branching and release strategy

OpenRigLogic follows a “live main, frozen stable” approach.

| OpenRigLogic branch | Stability |
| ---- | ---- |
| `main` | Experimental |
| `5.8` | Production-Ready |

### `main` branch

Most active development happens on the `main` branch. This branch is where new features are integrated and tested. We make it available for battle-hardened developers eager to test new features or work in lock-step with us.

If you choose to work in this branch, be aware that it is likely to be ahead of the branches for the current official release and the next upcoming release. Therefore, content and code that you create to work with the `main` branch may not be compatible with public releases until we create a new branch directly for a future official release.

This branch is best for developers who want the newest OpenRigLogic features.

### Stable branches (`5.8`, …)

Numbered branches identify past and upcoming official releases. Once a stable branch is created, it reflects the validated state of the libraries at that engine release. These branches are immutable snapshots with the exception of hotfix changes for critical fixes, and are recommended for production projects-use the branch that best matches your compatibility requirements.

## Supported platforms

OpenRigLogic supports a wide range of platforms that includes console platforms and mobile devices in addition to Windows, Linux, and macOS. 

Platform independence is a critical design feature. Real-time performance on each device is equally important.

## Dependencies

### Required

* **C++ Compiler** (supporting C++11 or higher)  
* [**CMake**](https://cmake.org/documentation/) (version 3.14+)

### Optional

These are only required if you enable specific build flags (e.g., tests, benchmarks, or language bindings).

| Dependency | Required for | Notes |
| ---- | ---- | ---- |
| [**SWIG**](https://www.swig.org/) | Python wrappers | Required at build time to generate the wrapper code |
| [**Python 3**](https://www.python.org/) (with Dev headers) | Python wrappers | Required at build time to compile the module, and at runtime to use it |
| [**Google Test**](https://github.com/google/googletest) | Unit tests | Downloaded automatically via CMake if tests are enabled |
| [**Google Benchmark**](https://github.com/google/benchmark) | Benchmarks | Downloaded automatically via CMake if benchmarks are enabled |

## Quick start guide

Clone the branch that matches your engine version:

```shell
git clone -b 5.8 https://github.com/EpicGames/OpenRigLogic.git
```

To track the latest changes from the main branch:

```shell
git clone https://github.com/EpicGames/OpenRigLogic.git
```

OpenRigLogic uses CMake to generate build scripts. From the repository root:

```shell
mkdir build
cd build
cmake ..
```

To build:

```shell
cmake --build .
```

## CMake configuration and build options

### AVX support

To enable the AVX-based joint calculation algorithm in RigLogic, set the `RL_BUILD_WITH_AVX` CMake option:

```shell
cmake .. -DRL_BUILD_WITH_AVX=ON
```

### Half-float optimization

To enable half-float backed joint storage in RigLogic, the target CPU must support the F16C extension, and the `RL_BUILD_WITH_HALF_FLOATS` CMake option must be set:

```shell
cmake .. -DRL_BUILD_WITH_HALF_FLOATS=ON
```

### Sanitizers

Sanitizers are well supported on Linux and provide the easiest and most accurate way to catch runtime programming errors. (Windows sanitizer support is less reliable.) Enable them at configure time:

```shell
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DENABLE_SANITIZERS=’address’
```

Then build and run a target as usual; the sanitizer reports errors detected at runtime. If sanitizer output is obscured, set the `ASAN_OPTIONS` and `ASAN_SYMBOLIZER_PATH` environment variables:

```shell
ASAN_OPTIONS=symbolize=1 ASAN_SYMBOLIZER_PATH=/usr/lib/llvm-6.0/bin/llvm-symbolizer ./rltests
```

Supported sanitizers:

- `address`  
- `memory`  
- `thread`  
- `leak`  
- `undefined`

## Tests

Tests can be run through CMake. When the active build system is MSBuild, the build configuration must be supplied on the command line:

```shell
ctest -C Debug
ctest -C Release
```

For make and ninja, only one build configuration is active at a time, so:

```shell
ctest
```

## Documentation

You can find the OpenRigLogic documentation [here](https://EpicGames.github.io/OpenRigLogic).

## Contributing

We welcome community contributions to OpenRigLogic. Before writing code, please read the [contributing guide](CONTRIBUTING.md), which describes the propose-first workflow (before you code) and DCO sign-off requirements.

## License

OpenRigLogic is released under the [MIT License](LICENSE).

## Community and support

For links to forums and information on submitting bug reports and feature requests please see [SUPPORT.md](SUPPORT.md)

See [SECURITY.md](SECURITY.md) for responsible disclosure of security related issues.
