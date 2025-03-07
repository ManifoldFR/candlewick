#include "candlewick/core/DefaultVertex.h"
#include "candlewick/utils/MeshData.h"
#include <gtest/gtest.h>

using namespace candlewick;

bool operator==(const DefaultVertex &lhs, const DefaultVertex &rhs) {
  return (lhs.pos == rhs.pos) && (lhs.normal == rhs.normal) &&
         (lhs.color == rhs.color);
}

GTEST_TEST(TestErasedBlob, default_vertex) {
  constexpr auto layout = meshLayoutFor<DefaultVertex>();
  EXPECT_TRUE(layout == layout);
  std::vector<DefaultVertex> vertexData;
  Uint64 size = 10;
  for (Uint64 i = 0; i < size; i++) {
    vertexData.push_back(
        {Float3::Random(), Float3::Zero(), 0.5f * Float4::Random()});
  }

  MeshData data(SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, vertexData);
  EXPECT_EQ(data.numVertices(), size);

  std::span<const DefaultVertex> view = data.viewAs<DefaultVertex>();
  EXPECT_EQ(view.size(), size);

  for (Uint64 i = 0; i < size; i++) {
    EXPECT_TRUE(view[i] == vertexData[i]);
    EXPECT_TRUE(vertexData[i].pos ==
                data.getAttribute<Float3>(i, VertexAttrib::Position));
    EXPECT_TRUE(vertexData[i].normal ==
                data.getAttribute<Float3>(i, VertexAttrib::Normal));
    EXPECT_TRUE(vertexData[i].color ==
                data.getAttribute<Float4>(i, VertexAttrib::Color0));
  }
}

struct alignas(16) CustomVertex {
  GpuVec4 pos;
  alignas(16) GpuVec3 color;
  alignas(16) GpuVec2 uv;
};
static_assert(IsVertexType<CustomVertex>, "Invalid vertex type.");

namespace candlewick {
template <> struct VertexTraits<CustomVertex> {
  static constexpr auto layout() {
    return MeshLayout{}
        .addBinding(0, sizeof(CustomVertex))
        .addAttribute(VertexAttrib::Position, 0,
                      SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,
                      offsetof(CustomVertex, pos))
        .addAttribute(VertexAttrib::Color0, 0,
                      SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
                      offsetof(CustomVertex, color))
        .addAttribute(VertexAttrib::TexCoord0, 0,
                      SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2,
                      offsetof(CustomVertex, uv));
  }
};
} // namespace candlewick

bool operator==(const CustomVertex &lhs, const CustomVertex &rhs) {
  return (lhs.pos == rhs.pos) && (lhs.color == rhs.color) && (lhs.uv == rhs.uv);
}

GTEST_TEST(TestErasedBlob, custom_vertex) {
  constexpr auto layout = meshLayoutFor<CustomVertex>();
  EXPECT_TRUE(layout == layout);
  EXPECT_TRUE(layout != meshLayoutFor<DefaultVertex>());
  EXPECT_EQ(sizeof(CustomVertex), layout.vertexSize());

  std::vector<CustomVertex> vertexData;
  const Uint64 size = 3;
  for (Uint64 i = 0; i < size; i++) {
    vertexData.push_back({Float4::Random(), Float3::Zero(), Float2::Random()});
  }

  MeshData data(SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, vertexData);
  EXPECT_EQ(data.numVertices(), size);

  std::span<const CustomVertex> view = data.viewAs<CustomVertex>();
  EXPECT_EQ(view.size(), size);

  for (Uint64 i = 0; i < size; i++) {
    EXPECT_TRUE(view[i] == vertexData[i]);
    EXPECT_TRUE(vertexData[i].pos ==
                data.getAttribute<Float4>(i, VertexAttrib ::Position));
    EXPECT_TRUE(vertexData[i].color ==
                data.getAttribute<Float3>(i, VertexAttrib::Color0));
    EXPECT_TRUE(vertexData[i].uv ==
                data.getAttribute<Float2>(i, VertexAttrib::TexCoord0));
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
