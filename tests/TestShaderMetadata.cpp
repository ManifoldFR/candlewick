#include "candlewick/core/Shader.h"
#include <gtest/gtest.h>

using namespace candlewick;

GTEST_TEST(TestShaderMetadata, uniform_buffer_only) {
  setShadersDirectory(CANDLEWICK_TEST_DATA_DIR);
  auto config = loadShaderMetadata("one_ubuffer.vert");
  EXPECT_EQ(config.stage, SDL_GPU_SHADERSTAGE_VERTEX);
  EXPECT_EQ(config.entry_point, "main");
  EXPECT_EQ(config.uniform_buffers, 1u);
  EXPECT_EQ(config.samplers, 0u);
  EXPECT_EQ(config.storage_textures, 0u);
  EXPECT_EQ(config.storage_buffers, 0u);
}

// entry_point is "fragmentMain" (non-"main") to exercise the name pass-through
GTEST_TEST(TestShaderMetadata, no_resources) {
  setShadersDirectory(CANDLEWICK_TEST_DATA_DIR);
  auto config = loadShaderMetadata("no_resources.frag");
  EXPECT_EQ(config.stage, SDL_GPU_SHADERSTAGE_FRAGMENT);
  EXPECT_EQ(config.entry_point, "fragmentMain");
  EXPECT_EQ(config.uniform_buffers, 0u);
  EXPECT_EQ(config.samplers, 0u);
  EXPECT_EQ(config.storage_textures, 0u);
  EXPECT_EQ(config.storage_buffers, 0u);
}

GTEST_TEST(TestShaderMetadata, samplers_and_ubuffer) {
  setShadersDirectory(CANDLEWICK_TEST_DATA_DIR);
  auto config = loadShaderMetadata("samplers_and_ubuffer.frag");
  EXPECT_EQ(config.stage, SDL_GPU_SHADERSTAGE_FRAGMENT);
  EXPECT_EQ(config.entry_point, "main");
  EXPECT_EQ(config.uniform_buffers, 1u);
  EXPECT_EQ(config.samplers, 2u);
  EXPECT_EQ(config.storage_textures, 0u);
  EXPECT_EQ(config.storage_buffers, 0u);
}

// --- Real compiled shader JSONs (shaders/compiled/) ---

// SSAOblur.frag: 1 Sampler2D (aoTex) + 1 UBO (BlurParams)
GTEST_TEST(TestShaderMetadataReal, SSAOblur_frag) {
  setShadersDirectory(CANDLEWICK_COMPILED_SHADERS_DIR);
  auto config = loadShaderMetadata("SSAOblur.frag");
  EXPECT_EQ(config.stage, SDL_GPU_SHADERSTAGE_FRAGMENT);
  EXPECT_EQ(config.entry_point, "main");
  EXPECT_EQ(config.uniform_buffers, 1u);
  EXPECT_EQ(config.samplers, 1u);
  EXPECT_EQ(config.storage_textures, 0u);
  EXPECT_EQ(config.storage_buffers, 0u);
}

// PbrBasic.frag: Sampler2DShadow + Sampler2D (SSAO) + Sampler2D (baseColorTex)
// + 3 UBOs (material, light, shadowAtlas)
GTEST_TEST(TestShaderMetadataReal, PbrBasic_frag) {
  setShadersDirectory(CANDLEWICK_COMPILED_SHADERS_DIR);
  auto config = loadShaderMetadata("PbrBasic.frag");
  EXPECT_EQ(config.stage, SDL_GPU_SHADERSTAGE_FRAGMENT);
  EXPECT_EQ(config.entry_point, "main");
  EXPECT_EQ(config.uniform_buffers, 3u);
  EXPECT_EQ(config.samplers, 3u);
  EXPECT_EQ(config.storage_textures, 0u);
  EXPECT_EQ(config.storage_buffers, 0u);
}

// Hud3dElement.vert: 1 UBO (mvp), no samplers
GTEST_TEST(TestShaderMetadataReal, Hud3dElement_vert) {
  setShadersDirectory(CANDLEWICK_COMPILED_SHADERS_DIR);
  auto config = loadShaderMetadata("Hud3dElement.vert");
  EXPECT_EQ(config.stage, SDL_GPU_SHADERSTAGE_VERTEX);
  EXPECT_EQ(config.entry_point, "main");
  EXPECT_EQ(config.uniform_buffers, 1u);
  EXPECT_EQ(config.samplers, 0u);
  EXPECT_EQ(config.storage_textures, 0u);
  EXPECT_EQ(config.storage_buffers, 0u);
}

// SolidColor.frag: no parameters at all
GTEST_TEST(TestShaderMetadataReal, SolidColor_frag) {
  setShadersDirectory(CANDLEWICK_COMPILED_SHADERS_DIR);
  auto config = loadShaderMetadata("SolidColor.frag");
  EXPECT_EQ(config.stage, SDL_GPU_SHADERSTAGE_FRAGMENT);
  EXPECT_EQ(config.entry_point, "main");
  EXPECT_EQ(config.uniform_buffers, 0u);
  EXPECT_EQ(config.samplers, 0u);
  EXPECT_EQ(config.storage_textures, 0u);
  EXPECT_EQ(config.storage_buffers, 0u);
}
