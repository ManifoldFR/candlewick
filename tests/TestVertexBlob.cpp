#include "candlewick/core/DefaultVertex.h"
#include "candlewick/utils/VertexDataBlob.h"
#include <gtest/gtest.h>

using namespace candlewick;

bool operator==(const DefaultVertex &lhs, const DefaultVertex &rhs) {
  return (lhs.pos == rhs.pos) && (lhs.normal == rhs.normal) &&
         (lhs.color == rhs.color);
}

GTEST_TEST(TestErasedBlob, default_vertex) {
  std::vector<DefaultVertex> vertexData;
  Uint64 size = 10;
  for (Uint64 i = 0; i < size; i++) {
    vertexData.push_back(
        {Float3::Random(), Float3::Zero(), 0.5f * Float4::Random()});
  }

  VertexDataBlob blob(vertexData);
  EXPECT_EQ(blob.size(), size);

  std::span<const DefaultVertex> view = blob.viewAs<DefaultVertex>();
  EXPECT_EQ(view.size(), size);

  for (Uint64 i = 0; i < size; i++) {
    EXPECT_TRUE(view[i] == vertexData[i]);
    EXPECT_TRUE(vertexData[i].pos == blob.getAttribute<Float3>(i, 0));
    EXPECT_TRUE(vertexData[i].normal == blob.getAttribute<Float3>(i, 1));
    EXPECT_TRUE(vertexData[i].color == blob.getAttribute<Float4>(i, 2));
  }
}

struct alignas(16) CustomVertex {
  GpuVec4 pos;
  alignas(16) GpuVec3 color;
  alignas(16) GpuVec2 uv;
};
static_assert(IsVertexType<CustomVertex>, "Invalid vertex type.");

template <> struct VertexTraits<CustomVertex> {
  constexpr static auto layout() {
    return MeshLayout{}
        .addBinding(0, sizeof(CustomVertex))
        .addAttribute("pos", 0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,
                      offsetof(CustomVertex, pos))
        .addAttribute("color", 1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
                      offsetof(CustomVertex, color))
        .addAttribute("uv", 2, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2,
                      offsetof(CustomVertex, uv));
  }
};

bool operator==(const CustomVertex &lhs, const CustomVertex &rhs) {
  return (lhs.pos == rhs.pos) && (lhs.color == rhs.color) && (lhs.uv == rhs.uv);
}

GTEST_TEST(TestErasedBlob, custom_vertex) {
  auto layout = meshLayoutFor<CustomVertex>();
  EXPECT_EQ(sizeof(CustomVertex), layout.vertexSize());

  std::vector<CustomVertex> vertexData;
  Uint64 size = 3;
  for (Uint64 i = 0; i < size; i++) {
    vertexData.push_back({Float4::Random(), Float3::Zero(), Float2::Random()});
  }

  VertexDataBlob blob(vertexData);
  EXPECT_EQ(blob.size(), size);

  std::span<const CustomVertex> view = blob.viewAs<CustomVertex>();
  EXPECT_EQ(view.size(), size);

  for (Uint64 i = 0; i < size; i++) {
    EXPECT_TRUE(view[i] == vertexData[i]);
    EXPECT_TRUE(vertexData[i].pos == blob.getAttribute<Float4>(i, 0));
    EXPECT_TRUE(vertexData[i].color == blob.getAttribute<Float3>(i, 1));
    EXPECT_TRUE(vertexData[i].uv == blob.getAttribute<Float2>(i, 2));
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
