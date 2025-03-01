#pragma once

#include "../core/MeshLayout.h"
#include <span>
#include <vector>
#include <SDL3/SDL_assert.h>

namespace candlewick {

/// \brief A type-erased container for vertex data.
class VertexDataBlob {
private:
  std::vector<char> m_data;
  MeshLayout m_layout;
  Uint64 m_actualSize;

public:
  VertexDataBlob() = default;
  explicit VertexDataBlob(std::vector<char> data, MeshLayout layout);

  template <IsVertexType V>
  VertexDataBlob(const std::vector<V> &vertices)
      : m_data(
            reinterpret_cast<const char *>(vertices.data()),
            reinterpret_cast<const char *>(vertices.data() + vertices.size())),
        m_layout(meshLayoutFor<V>()), m_actualSize(vertices.size()) {
    SDL_assert(meshLayoutFor<V>().vertexSize() == sizeof(V));
  }

  template <typename U> std::span<const U> viewAs() const {
    const U *begin = reinterpret_cast<const U *>(m_data.data());
    return std::span<const U>(begin, size());
  }

  template <typename U> std::span<U> viewAs() {
    U *begin = reinterpret_cast<U *>(m_data.data());
    return std::span<U>(begin, size());
  }

  /// \brief Access an attribute. Use this when the underlying vertex data type
  /// is unknown.
  template <typename T>
  T &getAttribute(const Uint64 vertexId, const SDL_GPUVertexAttribute &attr) {
    SDL_assert(vertexId < m_actualSize);
    const Uint32 vtxSize = m_layout.vertexSize();
    T *ptr =
        reinterpret_cast<T *>(m_data.data() + vertexId * vtxSize + attr.offset);
    return *ptr;
  }

  template <typename T>
  T &getAttribute(const Uint64 vertexId, VertexAttrib loc) {
    auto attr = m_layout.getAttribute(loc);
    if (!attr) {
      throw std::runtime_error("Vertex attribute " +
                               std::to_string(Uint16(loc)) + "not found.");
    }
    return this->getAttribute<T>(vertexId, *attr);
  }

  /// \brief Raw ptr to underlying data.
  char *data() { return m_data.data(); }
  /// \copybrief data()
  const char *data() const { return m_data.data(); }

  const MeshLayout &layout() const { return m_layout; }

  Uint64 size() const { return m_actualSize; }

  explicit operator std::vector<char>() const {
    std::vector<char> out(m_data.begin(), m_data.end());
    return out;
  }
};

inline VertexDataBlob::VertexDataBlob(std::vector<char> data, MeshLayout layout)
    : m_data(std::move(data)), m_layout(std::move(layout)),
      m_actualSize(m_data.size() / layout.vertexSize()) {}

} // namespace candlewick
