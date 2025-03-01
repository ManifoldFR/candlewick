#pragma once

#include <SDL3/SDL_gpu.h>
#include "math_types.h"
#include "../third-party/magic_enum.hpp"

inline bool operator==(const SDL_GPUVertexBufferDescription &lhs,
                       const SDL_GPUVertexBufferDescription &rhs) {
#define _c(field) lhs.field == rhs.field
  return _c(slot) && _c(pitch) && _c(input_rate) && _c(instance_step_rate);
#undef _c
}

inline bool operator==(const SDL_GPUVertexAttribute &lhs,
                       const SDL_GPUVertexAttribute &rhs) {
#define _c(field) lhs.field == rhs.field
  return _c(location) && _c(buffer_slot) && _c(format) && _c(offset);
#undef _c
}

namespace candlewick {

constexpr Uint64 vertexElementSize(SDL_GPUVertexElementFormat format) {
  switch (format) {
  case SDL_GPU_VERTEXELEMENTFORMAT_INT:
    return sizeof(Sint32);
  case SDL_GPU_VERTEXELEMENTFORMAT_INT2:
    return sizeof(Sint32[2]);
  case SDL_GPU_VERTEXELEMENTFORMAT_INT3:
    return sizeof(Sint32[3]);
  case SDL_GPU_VERTEXELEMENTFORMAT_INT4:
    return sizeof(Sint32[4]);
  case SDL_GPU_VERTEXELEMENTFORMAT_UINT:
    return sizeof(Uint32);
  case SDL_GPU_VERTEXELEMENTFORMAT_UINT2:
    return sizeof(Uint32[2]);
  case SDL_GPU_VERTEXELEMENTFORMAT_UINT3:
    return sizeof(Uint32[3]);
  case SDL_GPU_VERTEXELEMENTFORMAT_UINT4:
    return sizeof(Uint32[4]);
  case SDL_GPU_VERTEXELEMENTFORMAT_FLOAT:
    return sizeof(float);
  case SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2:
    return sizeof(float[2]);
  case SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3:
    return sizeof(float[3]);
  case SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4:
    return sizeof(float[4]);
  default:
    return 0u;
  }
}

/// \brief Fixed vertex attributes.
///
/// Each value of this enum maps to a specific input location in the shaders.
/// This defined an implicit contract between the engine and the shaders.
enum class VertexAttrib : Uint16 {
  Position,
  Normal,
  Tangent,
  Bitangent,
  Color0,
  Color1,
  TexCoord0,
  TexCoord1,
};

/// \brief Struct which defines the layout of a mesh's vertices.
///
/// This is used to build both rasterization pipeline create a Mesh.
/// \sa Mesh
/// \sa MeshData
class MeshLayout {
public:
  static constexpr size_t MAX_ATTRIBUTES =
      magic_enum::enum_count<VertexAttrib>();
  static constexpr size_t MAX_BINDINGS = 8ul;

  constexpr MeshLayout()
      : m_bindingCount(0), m_attrCount(0), m_bufferDescs{}, m_attrs{},
        m_totalVertexSize(0) {}

  /// \brief Add a binding (i.e. a vertex binding) for the mesh.
  ///
  /// Calling this multiple times allows to describe a mesh layout where e.g.
  /// positions and normals are non-interleaved and should be uploaded to
  /// different vertex buffers.
  /// \param slot Index for vertex buffer
  /// \param size Equivalent of pitch in SDL_gpu, size of consecutive elements
  /// of the vertex buffer, i.e. sizeof(Vertex) if your vertices are of some
  /// type Vertex.
  constexpr MeshLayout &addBinding(Uint32 slot, Uint32 pitch) {
    if (slot < MAX_BINDINGS) {
      m_bufferDescs[slot] = {slot, pitch, SDL_GPU_VERTEXINPUTRATE_VERTEX, 0u};
      if (slot >= m_bindingCount) {
        m_bindingCount++;
      }
    }
    return *this;
  }

  /// \brief Add a vertex attribute.
  /// \param name The vertex attribute name.
  /// \param loc Location index in the vertex shader.
  /// \param binding Binding slot of the corresponding vertex buffer.
  /// \param format Format of the vertex attribute.
  /// \param offset Byte offset of the attribute relative to the start of the
  /// vertex buffer element.
  constexpr MeshLayout &addAttribute(VertexAttrib loc, Uint32 binding,
                                     SDL_GPUVertexElementFormat format,
                                     Uint32 offset) {
    const Uint16 _loc = static_cast<Uint16>(loc);
    if (m_attrCount < MAX_ATTRIBUTES) {
      m_attrs[m_attrCount++] = {_loc, binding, format, offset};
      assert(binding < m_bindingCount);
    }
    const Uint32 attrSize =
        math::roundUpTo16(Uint32(vertexElementSize(format)));
    m_totalVertexSize = std::max(m_totalVertexSize, offset + attrSize);
    return *this;
  }

  const SDL_GPUVertexAttribute *getAttribute(VertexAttrib loc) const {
    for (size_t i = 0; i < MeshLayout::MAX_ATTRIBUTES; i++) {
      if (m_attrs[i].location == static_cast<Uint16>(loc)) {
        return &m_attrs[i];
      }
    }
    return nullptr;
  }

  /// \brief Cast to the SDL_GPU vertex input state struct, used to create
  /// pipelines.
  /// \warning The data here only references data internal to MeshLayout and
  /// *not* copied. It must stay in scope until the pipeline is created.
  SDL_GPUVertexInputState toVertexInputState() const {
    return {
        m_bufferDescs,
        m_bindingCount,
        m_attrs,
        m_attrCount,
    };
  }

  constexpr bool operator==(const MeshLayout &other) const {
    if (m_bindingCount != other.m_bindingCount &&
        m_attrCount != other.m_attrCount)
      return false;

    for (Uint16 i = 0; i < m_bindingCount; i++) {
      if (m_bufferDescs[i] != other.m_bufferDescs[i])
        return false;
    }
    for (Uint16 i = 0; i < m_attrCount; i++) {
      if (m_attrs[i] != other.m_attrs[i])
        return false;
    }
    return true;
  }

  /// \brief Number of vertex buffers.
  constexpr Uint32 numBuffers() const { return m_bindingCount; }
  /// \brief Number of vertex attributes.
  constexpr Uint32 numAttributes() const { return m_attrCount; }
  /// \brief Total size of a vertex (in bytes).
  /// \todo Make this compatible with multiple vertex bindings.
  constexpr Uint32 vertexSize() const { return m_totalVertexSize; }
  /// \brief Size of mesh indices (in bytes).
  constexpr Uint32 indexSize() const { return sizeof(Uint32); }

  Uint16 m_bindingCount;
  Uint16 m_attrCount;
  SDL_GPUVertexBufferDescription m_bufferDescs[MAX_BINDINGS];
  SDL_GPUVertexAttribute m_attrs[MAX_ATTRIBUTES];

private:
  Uint32 m_totalVertexSize;
};

/// \brief Validation function. Checks if a MeshLayout produces invalid data for
/// a Mesh.
inline bool validateMeshLayout(const MeshLayout &layout) {
  SDL_GPUVertexInputState inputState = layout.toVertexInputState();
  return (inputState.num_vertex_buffers > 0) &&
         (inputState.vertex_buffer_descriptions) &&
         (inputState.num_vertex_attributes > 0) &&
         (inputState.vertex_attributes);
}

/// \brief Basic concept checking if type V has the correct layout and alignment
/// requirements to be a vertex element.
template <typename V>
concept IsVertexType = std::is_standard_layout_v<V> && (alignof(V) == 16);

template <IsVertexType V> struct VertexTraits;

/// \brief Shortcut for extracting layout from compile-time struct.
template <IsVertexType V> constexpr MeshLayout meshLayoutFor() {
  return VertexTraits<V>::layout();
}

} // namespace candlewick
