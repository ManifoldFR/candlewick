#pragma once

#include "Utils.h"
#include "../core/MeshLayout.h"
#include "../core/MaterialUniform.h"
#include "../core/Tags.h"

#include <span>
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

struct MeshData : MeshDataBase<MeshData> {
private:
  std::vector<char> m_vertexData; //< Type-erased vertex data
  Uint32 m_numVertices;           //< Actual number of vertices
  Uint32 m_vertexSize;            //< Size of an individual vertex (in bytes)
public:
  using IndexType = Uint32;
  SDL_GPUPrimitiveType primitiveType; //< Geometry primitive for the mesh
  MeshLayout layout;                  //< Handle to the MeshLayout
  std::vector<IndexType> indexData;   //< Indices for indexed mesh. Optional.
  PbrMaterial material;               //< PBR material

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
  static MeshData copy(const MeshData &other) { return MeshData{other}; };

  /// \brief Number of individual vertices.
  Uint32 numVertices() const noexcept { return m_numVertices; }
  /// \brief Size of each vertex, in bytes.
  Uint32 vertexSize() const noexcept { return m_vertexSize; }
  /// \brief Size of the vertex data, in bytes.
  Uint64 vertexBytes() const noexcept { return m_vertexData.size(); }

  template <typename U> std::span<const U> viewAs() const {
    const U *begin = reinterpret_cast<const U *>(m_vertexData.data());
    return std::span<const U>(begin, m_numVertices);
  }

  template <typename U> std::span<U> viewAs() {
    U *begin = reinterpret_cast<U *>(m_vertexData.data());
    return std::span<U>(begin, m_numVertices);
  }

  /// \brief Access an attribute. Use this when the underlying vertex data type
  /// is unknown.
  template <typename T>
  T &getAttribute(const Uint64 vertexId, const SDL_GPUVertexAttribute &attr) {
    SDL_assert(vertexId < m_numVertices);
    const Uint32 vtxSize = layout.vertexSize();
    T *ptr = reinterpret_cast<T *>(m_vertexData.data() + vertexId * vtxSize +
                                   attr.offset);
    return *ptr;
  }

  template <typename T>
  T &getAttribute(const Uint64 vertexId, VertexAttrib loc) {
    auto attr = layout.getAttribute(loc);
    if (!attr) {
      throw std::runtime_error("Vertex attribute " +
                               std::to_string(Uint16(loc)) + "not found.");
    }
    return this->getAttribute<T>(vertexId, *attr);
  }

  std::span<const char> vertexData() const { return m_vertexData; }

  /// \brief Raw ptr to underlying vertex data.
  char *data() { return m_vertexData.data(); }
  /// \copybrief data()
  const char *data() const { return m_vertexData.data(); }

private:
  MeshData(const MeshData &) = default;
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

void uploadMeshToDevice(const Device &device, const Mesh &mesh,
                        const MeshData &meshData);

inline std::vector<PbrMaterial>
extractMaterials(std::span<const MeshData> meshDatas) {
  std::vector<PbrMaterial> out;
  out.reserve(meshDatas.size());
  for (size_t i = 0; i < meshDatas.size(); i++) {
    out.push_back(meshDatas[i].material);
  }
  return out;
}

} // namespace candlewick
