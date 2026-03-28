#include "candlewick/core/DefaultVertex.h"
#include "candlewick/core/Shader.h"
#include "candlewick/utils/MeshData.h"
#include "candlewick/utils/LoadMesh.h"
#include "candlewick/utils/LoadMaterial.h"
#include <gtest/gtest.h>

using namespace candlewick;

// -- DefaultVertex now includes texCoord --

GTEST_TEST(TestTextureVertex, default_vertex_has_texcoord) {
  auto layout = meshLayoutFor<DefaultVertex>();

  // TexCoord0 attribute should be present in the layout
  const auto *attr = layout.getAttribute(VertexAttrib::TexCoord0);
  ASSERT_NE(attr, nullptr);
  EXPECT_EQ(attr->format, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2);
  EXPECT_EQ(attr->offset, offsetof(DefaultVertex, texCoord));
}

GTEST_TEST(TestTextureVertex, texcoord_strided_access) {
  std::vector<DefaultVertex> vertexData;
  const Uint32 size = 5;
  for (Uint32 i = 0; i < size; i++) {
    DefaultVertex v{};
    v.pos = Float3::Ones() * float(i);
    v.texCoord = GpuVec2{float(i) * 0.1f, float(i) * 0.2f};
    vertexData.push_back(v);
  }

  MeshData data(SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, vertexData);
  EXPECT_EQ(data.numVertices(), size);

  // Access texCoord via strided view
  auto uv_view = data.getAttribute<GpuVec2>(VertexAttrib::TexCoord0);
  EXPECT_EQ(uv_view.size(), size);
  for (Uint32 i = 0; i < size; i++) {
    EXPECT_FLOAT_EQ(uv_view[i].x(), float(i) * 0.1f);
    EXPECT_FLOAT_EQ(uv_view[i].y(), float(i) * 0.2f);
  }
}

// -- MeshData stores baseColorTexturePath --

GTEST_TEST(TestTextureSupport, meshdata_texture_path_default_empty) {
  std::vector<DefaultVertex> vertexData(3);
  MeshData data(SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, vertexData);
  EXPECT_TRUE(data.baseColorTexturePath.empty());
}

GTEST_TEST(TestTextureSupport, meshdata_texture_path_settable) {
  std::vector<DefaultVertex> vertexData(3);
  MeshData data(SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, vertexData);
  data.baseColorTexturePath = "/some/path/texture.png";
  EXPECT_EQ(data.baseColorTexturePath, "/some/path/texture.png");
}

// -- getBaseColorTexturePath with null returns empty --

GTEST_TEST(TestTextureSupport, null_material_returns_empty_path) {
  std::string path = getBaseColorTexturePath(nullptr);
  EXPECT_TRUE(path.empty());
}

// -- Loading an OBJ without textures: baseColorTexturePath is empty --

GTEST_TEST(TestTextureSupport, obj_mesh_no_texture_path) {
  std::vector<MeshData> meshDatas;
  auto ret = loadSceneMeshes(CANDLEWICK_MESH_ASSETS_DIR "/cube.obj", meshDatas);
  ASSERT_EQ(ret, mesh_load_retc::OK);
  ASSERT_FALSE(meshDatas.empty());

  for (const auto &md : meshDatas) {
    EXPECT_TRUE(md.baseColorTexturePath.empty())
        << "OBJ meshes should not have texture paths";
  }
}

// -- Shader metadata: PbrBasic.frag now has 3 samplers --

GTEST_TEST(TestTextureShaders, PbrBasic_frag_has_base_color_sampler) {
  setShadersDirectory(CANDLEWICK_COMPILED_SHADERS_DIR);
  auto config = loadShaderMetadata("PbrBasic.frag");
  EXPECT_EQ(config.samplers, 3u)
      << "PbrBasic.frag should have 3 samplers: shadow, SSAO, baseColorTex";
}

// -- Shader metadata: PbrTransparent.frag now has 1 sampler --

GTEST_TEST(TestTextureShaders, PbrTransparent_frag_has_base_color_sampler) {
  setShadersDirectory(CANDLEWICK_COMPILED_SHADERS_DIR);
  auto config = loadShaderMetadata("PbrTransparent.frag");
  EXPECT_EQ(config.stage, SDL_GPU_SHADERSTAGE_FRAGMENT);
  EXPECT_EQ(config.entry_point, "main");
  EXPECT_EQ(config.uniform_buffers, 2u);
  EXPECT_EQ(config.samplers, 1u)
      << "PbrTransparent.frag should have 1 sampler: baseColorTex";
}

// -- Shader metadata: PbrBasic.vert accepts texCoord input --

GTEST_TEST(TestTextureShaders, PbrBasic_vert_metadata) {
  setShadersDirectory(CANDLEWICK_COMPILED_SHADERS_DIR);
  auto config = loadShaderMetadata("PbrBasic.vert");
  EXPECT_EQ(config.stage, SDL_GPU_SHADERSTAGE_VERTEX);
  EXPECT_EQ(config.uniform_buffers, 2u);
  EXPECT_EQ(config.samplers, 0u);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
