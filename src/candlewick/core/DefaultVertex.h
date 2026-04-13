#pragma once

#include "../core/math_types.h"
#include "../core/MeshLayout.h"

namespace candlewick {

struct alignas(16) DefaultVertex {
  GpuVec3 pos;
  alignas(16) GpuVec3 normal;
  alignas(16) GpuVec4 color;
  alignas(16) GpuVec3 tangent;
  alignas(8) GpuVec2 texCoord;
};
static_assert(IsVertexType<DefaultVertex>, "");

template <> struct VertexTraits<DefaultVertex> {
  static auto layout() {
    return MeshLayout{}
        .addBinding(0, sizeof(DefaultVertex))
        .addAttribute(VertexAttrib::Position, 0,
                      SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
                      offsetof(DefaultVertex, pos))
        .addAttribute(VertexAttrib::Normal, 0,
                      SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
                      offsetof(DefaultVertex, normal))
        .addAttribute(VertexAttrib::Color0, 0,
                      SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,
                      offsetof(DefaultVertex, color))
        .addAttribute(VertexAttrib::Tangent, 0,
                      SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
                      offsetof(DefaultVertex, tangent))
        .addAttribute(VertexAttrib::TexCoord0, 0,
                      SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2,
                      offsetof(DefaultVertex, texCoord));
  }
};

} // namespace candlewick
