# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.11.0] - 2026-02-26

### Added

- Expose `Visualizer.toggleGui(value=None)` (https://github.com/Simple-Robotics/candlewick/pull/102)
- Add option to set SSAO effect kernel size in Visualizer and RobotScene (C++/Python) (https://github.com/Simple-Robotics/candlewick/pull/105)
- Support for textures in rendering (https://github.com/Simple-Robotics/candlewick/pull/111)

### Changed

- Move cpp examples to their own directory (https://github.com/Simple-Robotics/candlewick/pull/103)
- Improved performance of SSAO effect (https://github.com/Simple-Robotics/candlewick/pull/105)
- Use `fmt::format` instead of `std::format` since we depend on spdlog (https://github.com/Simple-Robotics/candlewick/pull/106)
- Fix instances of not using formatting in call to `terminate_with_message` (https://github.com/Simple-Robotics/candlewick/pull/106)
- Migrate all shaders from GLSL to [Slang](https://shader-slang.com/), replacing the `glslc` + `shadercross` pipeline with `slangc` alone; update `process_shaders.py` accordingly ()
- Shader metadata parser now reads slangc reflection JSON; add unit tests against both hand-crafted and real compiled fixtures ()

### Fixed

- Fixed visual effects with SSAO + MSAA. Have SSAO sample from resolved multisampled depth texture (https://github.com/Simple-Robotics/candlewick/pull/105)
- Fix WBOIT transparent shader formulas, fixing transparency visual appearance and alpha channel in PNG screenshots (https://github.com/Simple-Robotics/candlewick/pull/110)

## [0.10.1] - 2026-01-19

- Fix issue in synchronous viewer (https://github.com/Simple-Robotics/candlewick/pull/98)

## [0.10.0] - 2025-08-20

### Added

- core/DepthAndShadowPass : add support for multisampled depth/shadow passes
- multibody : add MSAA support in `RobotScene` and `Visualizer`
- core/RenderContext.h : implement move ctor and assignment op
- multibody/Visualizer : working high DPI support
- core/GraphicsPipeline : store pipeline metadata
- core/Texture.h : add comparison operator
- core/Texture.h : add `sampleCount()` getter
- core : add RAII class `GraphicsPipeline`
- multibody : add wireframe mode switch in Visualizer GUI
- multibody/RobotDebug : add external forces to debug elts GUI
- core/DebugScene : add getters for subsystems

### Changed

- multibody/RobotScene : move `invalid_enum()` back to cpp file
- multibody/RobotScene : always render to G-buffer normal map
- core: move header `LoadCoalGeometries.h` and its functions to library core
- multibody/Visualizer : rename a data member in Config
- core/Device.h : implement move assignment operator
- core/RenderContext : introduce intermediate color render target, start introducing MSAA texture buffers
- core : move contents on `Scene.h` into `Core.h` header
- multibody : better graphics pipeline management using RAII class
- gui : Move all GUI functions into "gui" namespaces

### Removed

- Remove deprecated header `candlewick/core/Renderer.h`
- Remove deprecated header `candlewick/multibody/LoadCoalPrimitives.h`

## [0.9.0] - 2025-08-12

- utils/VideoRecorder : check filename extension (must be ".mp4")
- core/RenderContext : add setSwapchainParameters()
- multibody : enable displaying forces applies to frame (with lifetime) (https://github.com/Simple-Robotics/candlewick/pull/93)
- core/DebugScene : add function to add arrow entities (https://github.com/Simple-Robotics/candlewick/pull/93)
- core : Add `LambertMaterial` (https://github.com/Simple-Robotics/candlewick/pull/92)
- multibody/Visualizer : remove default world triad (https://github.com/Simple-Robotics/candlewick/pull/92)
  * (user must add it explicitly)
- multibody/Visualizer : pass scale to addFrameViz() (https://github.com/Simple-Robotics/candlewick/pull/92)
- core/DebugScene : ensure we re-use `Mesh` objects effectively (https://github.com/Simple-Robotics/candlewick/pull/92)

## [0.8.0] - 2025-07-25

### Added

- Added tools for loading models from a given description struct `RobotSpec` (https://github.com/Simple-Robotics/candlewick/pull/37)
- Added Candlewick visualizer runtime, to be used to (up)load Pinocchio models and submit states to be displayed asynchronously. (https://github.com/Simple-Robotics/candlewick/pull/37)
- Embed Inter Medium font into the application (https://github.com/Simple-Robotics/candlewick/pull/37)
- Add `RobotDebugSystem::reload()` function to switch out models

### Changed

- CMake : sync jrl-cmakemodules to new release 1.0.0 (https://github.com/Simple-Robotics/candlewick/pull/37)
- CMake : change option `BUILD_PYTHON_BINDINGS` to `BUILD_PYTHON_INTERFACE` (https://github.com/Simple-Robotics/candlewick/pull/37)
- multibody/Visualizer : `stopRecording()` now returns a flag (https://github.com/Simple-Robotics/candlewick/pull/37)
- core/Components : add `MeshMaterialComponent::hasTransparency()`
- multibody/RobotScene : allow reloading models, decouple model loading from render pipeline creation
- core/DepthAndShadowPass : add `ShadowMapPass::initialized()`
- core | multibody : rework of debug subsystem API, allow reloading RobotDebugSystem
- multibody/Visualizer : implement `loadViewerModel()`, allow it to actually reload models

### Fixed

- `core/Device.h` : fix release() function
- Python examples : wrap some import statements in try/except statements (https://github.com/Simple-Robotics/candlewick/pull/37)

### Removed

- Remove dependency on `robot_descriptions_cpp` (https://github.com/Simple-Robotics/candlewick/pull/37)

## [0.7.0] - 2025-07-04

### Added

- bindings/python : add expose-debug.cpp, expose DebugMeshComponent

### Changed

- slightly rework file dialog GUI
- core/DebugScene : make setupPipelines() private, do not use optional
- multibody/Visualizer/GUI : display info for robot debug frames

### Fixed

- fix screenshot PNG color channel ordering

## [0.6.0] - 2025-06-19

### Changed

- multibody/Visualizer : expose 'device'
- core : make `renderShadowPassFromFrustum()` work properly

### Removed

- core : remove using-decl of `coal::OBB`

## [0.5.0] - 2025-06-13

### Added

- bindings/python : expose video recorder settings
- bindings/python : add function `hasFfmpegSupport()` to check if... library was built with FFmpeg support
- pixi : separate `pinocchio` and `ffmpeg` features, add to CI build matrix

### Changed

- multibody/Visualizer : expose video settings directly instead of passing fps parameter
- cmake : allow user to turn off FFmpeg support/video recording features even if FFmpeg was detected (https://github.com/Simple-Robotics/candlewick/pull/83)

### Fixed

- support for Coal's current `devel` branch (remove incompatible fwd-declarations, just include `<coal/shape/geometric_shapes.h>`) (https://github.com/Simple-Robotics/candlewick/pull/81)
- Make FFmpeg actually optional: add option to turn off support even with FFmpeg installed, fix building with support turned off (https://github.com/Simple-Robotics/candlewick/pull/83)
- multibody/LoadCoalGeometries : fix setting the shape parameters for Capsule primitive (https://github.com/Simple-Robotics/candlewick/pull/84)
- multibody::loadGeometryObject() : always override material for `OT_GEOM` geometries (https://github.com/Simple-Robotics/candlewick/pull/84)

## [0.4.0] - 2025-06-10

### Added

- multibody/Visualizer : add frame visualization
- core/DebugScene : clean up entities with `DebugMeshComponent`
- core/DebugScene : add move ctor, explicitly delete move assignment op

### Changed

- core/DebugScene : pass `Float3` scale to `addTriad()`
- multibody : deprecate header `<candlewick/multibody/Components.h>`
- rename macro `CDW_ASSERT` to `CANDLEWICK_ASSERT` (for consistency)
- multibody/Visualizer : make more mouse buttons configurable
- docs : fix linking to pinocchio docs

### Removed

- remove default plane from Visualizer class

**Python**
- Removed `multibody` submodule. All symbols included in the main module. (**BREAKING CHANGE**)
- Fix stubs being generated for non-extension module classes/functions

## [0.3.1] - 2025-06-05

### Fixed

- utils/VideoRecorder.cpp : fix `m_frameCounter` not being initialized
- bindings/python : fix recorder context helper

## [0.3.0] - 2025-06-05

### Added

- multibody/Visualizer : pass fps to startRecording()

### Changed

- core : rename header `{ Renderer.h => RenderContext.h }`
- multibody : remove world scene bounds from RobotScene
- `Visualizer` : add `worldSceneBounds` data member
- python : expose `Visualizer::worldSceneBounds`

## [0.2.0] - 2025-05-31

### Fixed

- core: `Renderer` : fix not cleaning up depth texture if non-null (*critical*)
- core : `DebugScene` : fix not setting pipeline handles to nullptr
- multibody/Visualizer : fix not releasing transfer buffers before destroying render context (*critical*)

### Changed

- core : set max number of directional lights to 4

## [0.1.1] - 2025-05-26

### Added

- core/GuiSystem : add overload for adding multiple GUI elements for `DirectionalLight`s (https://github.com/Simple-Robotics/candlewick/pull/69)
- core/CommandBuffer.h : add overload for C-style arrays to uniform-pushing methods (https://github.com/Simple-Robotics/candlewick/pull/69)
- core/Shader : store shader stage (https://github.com/Simple-Robotics/candlewick/pull/69)
- core/DepthAndShadowPass.h : handle multiple shadow maps using a texture atlas (https://github.com/Simple-Robotics/candlewick/pull/69)
- multibody : handle two-light setup w/ shadow mapping (https://github.com/Simple-Robotics/candlewick/pull/69)
- multibody/Visualizer : use `H` key to toggle GUI (https://github.com/Simple-Robotics/candlewick/pull/70)
- shaders : Soft shadows in PBR using percentage-closer filtering (PCF) (https://github.com/Simple-Robotics/candlewick/pull/71)
- core : Tighter shadow frustum around the world-scene AABB (https://github.com/Simple-Robotics/candlewick/pull/71)
- core : add `getAABBCorners()` util function in `Collision.h` header (https://github.com/Simple-Robotics/candlewick/pull/71)

## [0.1.0] - 2025-05-21

### Added

- core : add `guiAddFileDialog` (https://github.com/Simple-Robotics/candlewick/pull/61)
- utils : refactor `VideoRecorder` ctor, add `open()` member function (https://github.com/Simple-Robotics/candlewick/pull/62)
- multibody/Visualizer add screenshot button in visualizer's GUI (https://github.com/Simple-Robotics/candlewick/pull/62)
- utils : add `generateScreenshotFilenameFromTimestamp()` (https://github.com/Simple-Robotics/candlewick/pull/63)
- multibody/Visualizer : GUI : add buttons to start video recording, add `takeScreenshot()` method (https://github.com/Simple-Robotics/candlewick/pull/63)
- utils: add VideoRecorder::close() API (to manually close recorder) (https://github.com/Simple-Robotics/candlewick/pull/61)
- multibody/Visualizer : add `startRecording(std::string_view)` and `stopRecording()` methods, expose to Python (https://github.com/Simple-Robotics/candlewick/pull/65)

### Changed

- utils : avoid public inclusion of `libavutil/pixfmt` header for video recorder (https://github.com/Simple-Robotics/candlewick/pull/61)
- core : `errors.h`: make terminate_with_message a template (with formatting arguments, etc) (https://github.com/Simple-Robotics/candlewick/pull/61)
- multibody/RobotDebug.cpp : fix velocity arrow direction (https://github.com/Simple-Robotics/candlewick/pull/61)
- utils : make `writeTextureToVideoFrame` a member function (https://github.com/Simple-Robotics/candlewick/pull/62)
- utils : rename `writeToFile` to `saveTextureToFile` (https://github.com/Simple-Robotics/candlewick/pull/62)
- utils/PixelFormatConversion.h : make bgra conversion function in-place (https://github.com/Simple-Robotics/candlewick/pull/63)
- utils & third-party : switch from `stb_image_write.h` to `fpng` for writing PNG files (https://github.com/Simple-Robotics/candlewick/pull/64)

### Fixed

- core : fix `CommandBuffer::submitAndAcquireFence()` not setting the internal pointer to null (https://github.com/Simple-Robotics/candlewick/pull/61)
- Fix all calls to `terminate_with_message()` (format string with `%`) (https://github.com/Simple-Robotics/candlewick/pull/62)

## [0.0.7] - 2025-05-17

### Added

- core : make DepthPassInfo an aggregate (https://github.com/Simple-Robotics/candlewick/pull/57)
- core : add `updateTransparencyClassification()` to tag an entity as opaque or untag it (https://github.com/Simple-Robotics/candlewick/pull/57)
- proper support for transparent objects (https://github.com/Simple-Robotics/candlewick/pull/58)
- shaders : add WBOIT composite shader, PBR transparent shader, `utils.glsl` util module, `pbr_lighting.glsl` (https://github.com/Simple-Robotics/candlewick/pull/58)
- core/`CommandBuffer` : type-safe wrappers for pushing uniforms, add `Raw` suffix to raw methods (https://github.com/Simple-Robotics/candlewick/pull/59)

### Changed

- remove `projMatrix` from PBR shader ubo (https://github.com/Simple-Robotics/candlewick/pull/58)
- multibody/RobotScene : early return if pipeline is nullptr (https://github.com/Simple-Robotics/candlewick/pull/58)
- multibody/RobotScene : reorganize pipelines (accomodate for transparent PBR shader)
- shaders : refactor basic PBR shader (move some functions to new `pbr_lighting.glsl`) (https://github.com/Simple-Robotics/candlewick/pull/58)
- interleave debug scene render between robot render system opaque and transparent passes (https://github.com/Simple-Robotics/candlewick/pull/59)
- revamp depth and shadow map pass classes (https://github.com/Simple-Robotics/candlewick/pull/60)
- core/Texture : do not store pointer to `Device` but raw `SDL_GPUDevice *` handle (https://github.com/Simple-Robotics/candlewick/pull/60)

## [0.0.6] - 2025-05-14

This is the first release to use a changelog.

### Removed

- Remove `MeshData loadCoalHeightField(const coal::CollisionGeometry &collGeom)`

### Added

- `RobotScene.h`: move invalid_enum() out to public header
- Add template class `strided_view<T>` for non-contiguous evenly-strided data
- `errors.h`: Use `string_view` for `_error_message_impl`
- Extend support for coal `CollisionGeometry` objects
  - Add support for `coal::GEOM_ELLIPSOID`
  - Add support for `coal::GEOM_CONVEX`
  - Add loader `loadCoalConvex()`

### Changed

- multibody : change signature and rename { guiPinocchioModelInfo() => guiAddPinocchioModelInfo() } ([#54](https://github.com/Simple-Robotics/candlewick/pull/54))
- core : pass wrap width to showCandlewickAboutWindow ([#54](https://github.com/Simple-Robotics/candlewick/pull/54))
- Visualizer: set next window pos ([#54](https://github.com/Simple-Robotics/candlewick/pull/54))
- core/GuiSystem.h : rename some free functions, group them in topic
- Read geometry object `meshColor` `meshScale` when updating robot scene ([#50](https://github.com/Simple-Robotics/candlewick/pull/50))
- Set minimum version of ffmpeg to 7.x ([#50](https://github.com/Simple-Robotics/candlewick/pull/50))
- Change signature of `loadCoalPrimitive()` to take `coal::ShapeBase`
- Change signature of `MeshData::getAttribute()` template member function to use strided view

### Fixed

- Visual bug (uninitialized data) when disabling SSAO


[Unreleased]: https://github.com/Simple-Robotics/candlewick/compare/v0.11.0...HEAD
[0.11.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.10.1...v0.11.0
[0.10.1]: https://github.com/Simple-Robotics/candlewick/compare/v0.10.0...v0.10.1
[0.10.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.9.0...v0.10.0
[0.9.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.8.0...v0.8.0
[0.8.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.5.0...v0.5.0
[0.5.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.3.1...v0.4.0
[0.3.1]: https://github.com/Simple-Robotics/candlewick/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/Simple-Robotics/candlewick/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/Simple-Robotics/candlewick/compare/v0.0.7...v0.1.0
[0.0.7]: https://github.com/Simple-Robotics/candlewick/compare/v0.0.6...v0.0.7
[0.0.6]: https://github.com/Simple-Robotics/candlewick/compare/v0.0.6...v0.0.6
[0.0.6]: https://github.com/Simple-Robotics/candlewick/compare/v0.0.5...v0.0.6
