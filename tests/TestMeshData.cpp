#include "candlewick/core/DefaultVertex.h"
#include "candlewick/utils/MeshData.h"
#include <gtest/gtest.h>

using namespace candlewick;

bool operator==(const DefaultVertex &lhs, const DefaultVertex &rhs) {
  return (lhs.pos == rhs.pos) && (lhs.normal == rhs.normal) &&
         (lhs.color == rhs.color) && (lhs.texCoord == rhs.texCoord);
}

struct alignas(16) CustomVertex {
  GpuVec4 pos;
  alignas(16) GpuVec3 color;
  alignas(16) GpuVec2 uv;
};
static_assert(IsVertexType<CustomVertex>, "Invalid vertex type.");

namespace candlewick {
template <> struct VertexTraits<CustomVertex> {
  static auto layout() {
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

GTEST_TEST(TestErasedBlob, default_vertex) {
  auto layout = meshLayoutFor<DefaultVertex>();
  EXPECT_EQ(layout, layout);
  std::vector<DefaultVertex> vertexData;
  const Uint64 size = 10;
  for (Uint64 i = 0; i < size; i++) {
    vertexData.push_back(
        {(i + 1) * Float3::Ones(), Float3::Zero(), 0.5f * Float4::Random()});
  }

  MeshData data(SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, vertexData);
  EXPECT_EQ(data.numVertices(), size);

  std::span<const DefaultVertex> view = data.viewAs<DefaultVertex>();
  EXPECT_EQ(view.size(), size);

  strided_view pos_view = data.getAttribute<Float3>(VertexAttrib::Position);
  strided_view nor_view = data.getAttribute<Float3>(VertexAttrib::Normal);
  strided_view col_view = data.getAttribute<Float4>(VertexAttrib::Color0);
  for (Uint64 i = 0; i < size; i++) {
    EXPECT_TRUE(view[i] == vertexData[i]);
    EXPECT_TRUE(vertexData[i].pos == pos_view[i]);
    EXPECT_TRUE(vertexData[i].normal == nor_view[i]);
    EXPECT_TRUE(vertexData[i].color == col_view[i]);
  }

  auto pos_attr = data.getAttribute<GpuVec3>(VertexAttrib::Position);
  EXPECT_EQ(pos_attr.size(), size);
  for (Uint64 i = 0; i < size; i++) {
    std::cout << i << " -- " << pos_attr[i].transpose() << std::endl;
    EXPECT_EQ(pos_attr[i], vertexData[i].pos);
  }
}

GTEST_TEST(TestErasedBlob, custom_vertex) {
  auto layout = meshLayoutFor<CustomVertex>();
  EXPECT_NE(layout, meshLayoutFor<DefaultVertex>());
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

  strided_view pos_view = data.getAttribute<Float4>(VertexAttrib::Position);
  strided_view col_view = data.getAttribute<Float3>(VertexAttrib::Color0);
  strided_view uv_view = data.getAttribute<Float2>(VertexAttrib::TexCoord0);
  for (Uint64 i = 0; i < size; i++) {
    EXPECT_EQ(view[i], vertexData[i]);
    EXPECT_EQ(vertexData[i].pos, pos_view[i]);
    EXPECT_EQ(vertexData[i].color, col_view[i]);
    EXPECT_EQ(vertexData[i].uv, uv_view[i]);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
