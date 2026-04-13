find_package(GTest REQUIRED)

function(add_candlewick_test filename)
  cmake_path(GET filename STEM name)
  add_executable(${name} ${filename})
  target_link_libraries(${name} PRIVATE candlewick_core GTest::gtest_main)
  target_link_libraries(${name} PRIVATE ${ARGN})
  add_test(NAME ${name} COMMAND ${name})
endfunction()

add_candlewick_test(TestMeshData.cpp)
add_candlewick_test(TestStrided.cpp)
add_candlewick_test(TestShaderMetadata.cpp)
target_compile_definitions(
  TestShaderMetadata
  PRIVATE
    CANDLEWICK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
    CANDLEWICK_COMPILED_SHADERS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../shaders/compiled"
)

add_candlewick_test(TestTextureSupport.cpp)
target_compile_definitions(
  TestTextureSupport
  PRIVATE
    CANDLEWICK_MESH_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../assets/meshes"
    CANDLEWICK_COMPILED_SHADERS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../shaders/compiled"
)
