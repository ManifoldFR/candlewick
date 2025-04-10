# Copyright (c) 2024-2025 ManifoldFR

ADD_PROJECT_DEPENDENCY(assimp REQUIRED)
ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED)
if(BUILD_PINOCCHIO_VISUALIZER)
  ADD_PROJECT_DEPENDENCY(hpp-fcl REQUIRED)
  ADD_PROJECT_DEPENDENCY(pinocchio REQUIRED)
endif()
ADD_PROJECT_DEPENDENCY(coal REQUIRED)
ADD_PROJECT_DEPENDENCY(nlohmann_json 3.11.3 REQUIRED)
ADD_PROJECT_DEPENDENCY(EnTT REQUIRED)
ADD_PROJECT_DEPENDENCY(magic_enum 0.9.7 CONFIG REQUIRED)
ADD_PROJECT_DEPENDENCY(
  FFmpeg
  COMPONENTS
  avformat
  avcodec
  swscale
)
find_package(ZeroMQ REQUIRED)

add_library(
  candlewick_core
  SHARED
  candlewick/core/Camera.cpp
  candlewick/core/CommandBuffer.cpp
  candlewick/core/DebugScene.cpp
  candlewick/core/DepthAndShadowPass.cpp
  candlewick/core/Device.cpp
  candlewick/core/math_util.cpp
  candlewick/core/errors.cpp
  candlewick/core/GuiSystem.cpp
  candlewick/core/Mesh.cpp
  candlewick/core/Renderer.cpp
  candlewick/core/Shader.cpp
  candlewick/core/Texture.cpp
  candlewick/core/debug/DepthViz.cpp
  candlewick/core/debug/Frustum.cpp
  candlewick/posteffects/ScreenSpaceShadows.cpp
  candlewick/posteffects/SSAO.cpp
  candlewick/utils/LoadMesh.cpp
  candlewick/utils/LoadMaterial.cpp
  candlewick/utils/MeshData.cpp
  candlewick/utils/MeshDataView.cpp
  candlewick/utils/MeshTransforms.cpp
  candlewick/utils/PixelFormatConversion.cpp
  candlewick/utils/WriteTextureToImage.cpp
  candlewick/primitives/Arrow.cpp
  candlewick/primitives/Capsule.cpp
  candlewick/primitives/Cone.cpp
  candlewick/primitives/Cube.cpp
  candlewick/primitives/Cylinder.cpp
  candlewick/primitives/Grid.cpp
  candlewick/primitives/Heightfield.cpp
  candlewick/primitives/Internal.cpp
  candlewick/primitives/Plane.cpp
  candlewick/primitives/Sphere.cpp
  candlewick/third-party/sbti_lib.cpp
  # imgui sources
  ${IMGUI_SRC}
)

target_link_libraries(
  candlewick_core
  PUBLIC
    SDL3::SDL3-shared
    assimp::assimp
    coal::coal
    magic_enum::magic_enum
    EnTT::EnTT
  PRIVATE imgui_headers nlohmann_json::nlohmann_json
)
target_compile_definitions(
  candlewick_core
  PUBLIC
    $<BUILD_INTERFACE:CANDLEWICK_SHADER_BIN_DIR="${CANDLEWICK_SHADERS_DIR}/compiled">
    $<INSTALL_INTERFACE:CANDLEWICK_SHADER_BIN_DIR="${CANDLEWICK_SHADER_INSTALL_DIR}/compiled">
)
target_include_directories(
  candlewick_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(FFmpeg_FOUND)
  message(
    STATUS
    "FFmpeg found (version ${FFmpeg_VERSION}). Building video module for candlewick_core."
  )
  target_compile_definitions(
    candlewick_core
    PUBLIC CANDLEWICK_WITH_FFMPEG_SUPPORT
  )
  target_link_libraries(
    candlewick_core
    PRIVATE FFmpeg::avformat FFmpeg::avcodec FFmpeg::swscale
  )
  target_sources(candlewick_core PRIVATE candlewick/utils/VideoRecorder.cpp)
endif()

install(
  TARGETS candlewick_core
  EXPORT ${TARGETS_EXPORT_NAME}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

add_library(${PROJECT_NAME} INTERFACE)
add_library(candlewick::candlewick ALIAS candlewick)
target_link_libraries(${PROJECT_NAME} INTERFACE candlewick_core)

if(BUILD_PINOCCHIO_VISUALIZER)
  file(GLOB multibody_sources candlewick/multibody/*.cpp)

  add_library(candlewick_multibody SHARED ${multibody_sources})
  target_link_libraries(
    candlewick_multibody
    PUBLIC
      candlewick_core
      pinocchio::pinocchio_default
      pinocchio::pinocchio_visualizers
    PRIVATE imgui_headers
  )
  target_link_libraries(${PROJECT_NAME} INTERFACE candlewick_multibody)

  install(
    TARGETS candlewick_multibody
    EXPORT ${TARGETS_EXPORT_NAME}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

  add_executable(candlewick_visualizer candlewick/runtime/main.cpp)
  target_link_libraries(candlewick_visualizer PRIVATE candlewick_multibody)
endif()

# install headers
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.h"
)
