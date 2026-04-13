#pragma once

#include "Utils.h"
#include "StridedView.h"
#include "../core/errors.h"
#include "../core/MeshLayout.h"
#include "../core/MaterialUniform.h"
#include "../core/Tags.h"

#include <span>
#include <string>
#include <SDL3/SDL_assert.h>
#include <SDL3/SDL_gpu.h>

namespace candlewick {

template <typename Derived> struct MeshDataBase {
  Derived &derived() { return static_cast<Derived &>(*this); }
  const Derived &derived() const { return static_cast<const Derived &>(*this); }

  Uint32 numVertices() const { return derived().numVertices(); }

  Uint32 numIndices() const {
    return static_cast<Uint32>(derived().indexData.size());
  }
  bool isIndexed() const { return numIndices() > 0; }
};

/// \brief A class to store type-erased vertex data and index data.
///
/// This is to be used as an intermediate data representation class to map to
/// Mesh objects.
///
class MeshData : public MeshDataBase<MeshData> {
  std::vector<char> m_vertexData; //< Type-erased vertex data
  Uint32 m_numVertices;           //< Actual number of vertices

  MeshData(const MeshData &) = default;

public:
  using IndexType = Uint32;
  SDL_GPUPrimitiveType primitiveType; //< Geometry primitive for the mesh.
  MeshLayout layout;                  //< %Mesh layout.
  std::vector<IndexType> indexData;   //< Indices for indexed mesh. Optional.
  PbrMaterial material;               //< Mesh material.
  std::string baseColorTexturePath;   //< Path to base color texture, if any.

  explicit MeshData(NoInitT);

  template <IsVertexType VertexT>
  explicit MeshData(SDL_GPUPrimitiveType primitiveType,
                    std::vector<VertexT> vertexData,
                    std::vector<IndexType> indexData = {});

  explicit MeshData(SDL_GPUPrimitiveType primitiveType,
                    const MeshLayout &layout, std::vector<char> vertexData,
                    std::vector<IndexType> indexData = {});

  MeshData(MeshData &&) noexcept = default;
  MeshData &operator=(MeshData &&) noexcept = default;
  MeshData &operator=(const MeshData &) noexcept = delete;

  /// \brief Explicit copy function, uses private copy ctor.
  [[nodiscard]] static MeshData copy(const MeshData &other) {
    return MeshData{other};
  };

  /// \brief Number of individual vertices.
  Uint32 numVertices() const noexcept { return m_numVertices; }
  /// \brief Size of each vertex, in bytes.
  Uint32 vertexSize() const noexcept { return layout.vertexSize(); }
  /// \brief Size of the overall vertex data, in bytes.
  Uint64 vertexBytes() const noexcept { return m_vertexData.size(); }

  /// \brief Obtain a typed view to the underlying vertex data.
  ///
  /// This should be used in tandem with the typed constructor.
  template <typename U> std::span<U> viewAs() {
    U *begin = reinterpret_cast<U *>(m_vertexData.data());
    return std::span<U>(begin, m_numVertices);
  }

  /// \copybrief viewAs().
  template <typename U> std::span<const U> viewAs() const {
    const U *begin = reinterpret_cast<const U *>(m_vertexData.data());
    return std::span<const U>(begin, m_numVertices);
  }

  /// \brief Access an attribute. Use this when the underlying vertex data type
  /// is unknown.
  template <typename T>
  [[nodiscard]] strided_view<T>
  getAttribute(const SDL_GPUVertexAttribute &attr) {
    const Uint32 stride = layout.vertexSize();
    auto ptr = m_vertexData.data() + attr.offset;
    return strided_view(reinterpret_cast<T *>(ptr), m_numVertices, stride);
  }

  template <typename T>
  [[nodiscard]] strided_view<const T>
  getAttribute(const SDL_GPUVertexAttribute &attr) const {
    const Uint32 stride = layout.vertexSize();
    auto ptr = m_vertexData.data() + attr.offset;
    return strided_view(reinterpret_cast<const T *>(ptr), m_numVertices,
                        stride);
  }

  template <typename T>
  [[nodiscard]] strided_view<T> getAttribute(VertexAttrib loc) {
    if (auto attr = layout.getAttribute(loc)) {
      return this->getAttribute<T>(*attr);
    }
    terminate_with_message("Vertex attribute {:d} not found.", Uint32(loc));
  }

  template <typename T>
  [[nodiscard]] strided_view<const T> getAttribute(VertexAttrib loc) const {
    if (auto attr = layout.getAttribute(loc)) {
      return this->getAttribute<T>(*attr);
    }
    terminate_with_message("Vertex attribute {:d} not found.", Uint32(loc));
  }

  std::span<const char> vertexData() const { return m_vertexData; }
};

template <IsVertexType VertexT>
MeshData::MeshData(SDL_GPUPrimitiveType primitiveType,
                   std::vector<VertexT> vertexData,
                   std::vector<IndexType> indexData)
    : MeshData(primitiveType, meshLayoutFor<VertexT>(),
               std::vector<char>{reinterpret_cast<char *>(vertexData.data()),
                                 reinterpret_cast<char *>(vertexData.data() +
                                                          vertexData.size())},
               std::move(indexData)) {}

/// \brief Convert MeshData to a GPU Mesh object. This creates the
/// required vertex buffer and index buffer (if required).
/// \warning This does *not* upload the mesh data to the vertex and index
/// buffers.
/// \sa uploadMeshToDevice()
[[nodiscard]] Mesh createMesh(const Device &device, const MeshData &meshData,
                              bool upload = false);

/// \brief Create a Mesh object from given mesh data, as a view into existing
/// vertex and index buffers.
/// \warning The constructed Mesh will **take ownership** of the buffers.
[[nodiscard]] Mesh createMesh(const Device &device, const MeshData &meshData,
                              SDL_GPUBuffer *vertexBuffer,
                              SDL_GPUBuffer *indexBuffer);

/// \brief Create a Mesh from a batch of MeshData.
/// \param[in] device GPU device
/// \param[in] meshDatas Batch of meshes
/// \param[in] upload Whether to upload the resulting Mesh to the device.
/// \sa createMesh()
/// \sa uploadMeshToDevice()
[[nodiscard]] Mesh createMeshFromBatch(const Device &device,
                                       std::span<const MeshData> meshDatas,
                                       bool upload);

/// \brief Upload the contents of a single, individual mesh to the GPU device.
///
/// This will upload the mesh data through a MeshView.
void uploadMeshToDevice(const Device &device, const MeshView &meshView,
                        const MeshData &meshData);

/// \copybrief uploadMeshToDevice().
void uploadMeshToDevice(const Device &device, const Mesh &mesh,
                        const MeshData &meshData);

inline void extractMaterials(std::span<const MeshData> meshDatas,
                             std::vector<PbrMaterial> &out) {
  for (size_t i = 0; i < meshDatas.size(); i++) {
    out.push_back(meshDatas[i].material);
  }
}

[[nodiscard]] inline std::vector<PbrMaterial>
extractMaterials(std::span<const MeshData> meshDatas) {
  std::vector<PbrMaterial> out;
  out.reserve(meshDatas.size());
  extractMaterials(meshDatas, out);
  return out;
}

} // namespace candlewick
