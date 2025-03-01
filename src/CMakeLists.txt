# Copyright (c) 2024-2025 ManifoldFR

find_package(assimp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(coal REQUIRED)
find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(EnTT REQUIRED)
find_package(FFmpeg COMPONENTS avformat avcodec swscale)

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
    Eigen3::Eigen
    coal::coal
    EnTT::EnTT
    imgui-headers
  PRIVATE nlohmann_json::nlohmann_json
)
target_compile_definitions(
  candlewick_core
  PUBLIC
    $<BUILD_INTERFACE:CANDLEWICK_SHADER_BIN_DIR="${CANDLEWICK_SHADERS_DIR}/compiled">
    $<INSTALL_INTERFACE:CANDLEWICK_SHADER_BIN_DIR="${CANDLEWICK_SHADER_INSTALL_DIR}/compiled">
)
target_include_directories(candlewick_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(FFmpeg_FOUND)
  message(STATUS "FFmpeg found. Building video module.")
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
target_link_libraries(${PROJECT_NAME} INTERFACE candlewick_core)
target_include_directories(${PROJECT_NAME} INTERFACE candlewick)

if(BUILD_PINOCCHIO_VISUALIZER)
  find_package(pinocchio REQUIRED)
  find_package(robot_descriptions_cpp REQUIRED)

  file(
    GLOB multibody_sources
    candlewick/multibody/*.cpp
    candlewick/multibody/internal/*.cpp
  )

  add_library(candlewick_multibody SHARED ${multibody_sources})
  target_link_libraries(
    candlewick_multibody
    PUBLIC
      candlewick_core
      robot_descriptions_cpp::robot_descriptions_cpp
      pinocchio::pinocchio_default
      pinocchio::pinocchio_visualizers
  )
  target_link_libraries(${PROJECT_NAME} INTERFACE candlewick_multibody)

  install(
    TARGETS candlewick_multibody
    EXPORT ${TARGETS_EXPORT_NAME}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
endif()

# install headers
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.h"
)
