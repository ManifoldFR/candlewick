#pragma once

#include "Core.h"
#include "Tags.h"
#include "MeshLayout.h"

#include <vector>
#include <span>
#include <SDL3/SDL_assert.h>
#include <entt/entity/registry.hpp>
#include <entt/entity/handle.hpp>

namespace candlewick {

/// \brief A view into a Mesh object.
///
/// Objects of this class are copyable and movable, since they only "borrow" the
/// buffers.
/// \warning A MeshView is expected to be non-empty: positive vertex count and
/// index count (if initial Mesh is indexed).
/// \sa Mesh
class MeshView {
  friend class Mesh;
  MeshView() noexcept = default;

public:
  /// Vertex buffers.
  std::vector<SDL_GPUBuffer *> vertexBuffers;
  /// Index buffer.
  SDL_GPUBuffer *indexBuffer;

  /// Vertex offsets, expressed in elements.
  Uint32 vertexOffset;
  /// Number of vertices in the mesh view.
  Uint32 vertexCount;

  /// Index offset (in elements).
  Uint32 indexOffset;
  /// Number of indices in the mesh view.
  Uint32 indexCount;

  bool isIndexed() const { return indexBuffer != nullptr; }

  MeshView(const MeshView &parent, Uint32 subVertexOffset,
           Uint32 vertexSubCount, Uint32 subIndexOffset, Uint32 indexSubCount);
};

/// \brief Handle class for meshes (vertex buffers and an optional index buffer)
/// on the GPU.
///
/// This class contains the layout, vertex (and index) count(s), and handles to
/// the GPU vertex and index buffers the Mesh references.
///
/// A Mesh **owns** its vertex and index buffers.
///
/// \sa MeshView
class Mesh {
  SDL_GPUDevice *m_device{nullptr};
  std::vector<MeshView> m_views;
  MeshLayout m_layout;

public:
  Uint32 vertexCount;
  Uint32 indexCount{0u};

  /// Vertex buffers the Mesh has its vertex data in.
  ///
  /// There may be multiple vertex buffers, depending on how the mesh data is
  /// laid out. For instance, some vertex attributes may be non-interleaved,
  /// e.g. the vertex positions, normals, and colors may be in different vertex
  /// buffers in GPU memory, instead of being laid out as
  /// `[pos0, norm0, col0, pos1, ...]`.
  std::vector<SDL_GPUBuffer *> vertexBuffers;

  /// Index buffer for the mesh's index data. If this is null, then the
  /// Mesh is considered to be *non*-indexed when it is bound or when draw
  /// commands are issued.
  SDL_GPUBuffer *indexBuffer{nullptr};

  explicit Mesh(const Device &device, const MeshLayout &layout);

  const MeshLayout &layout() const { return m_layout; }

  Mesh(NoInitT);
  Mesh(const Mesh &) = delete;
  Mesh(Mesh &&other) noexcept;

  Mesh &operator=(const Mesh &) = delete;
  Mesh &operator=(Mesh &&other) noexcept;

  const MeshView &view(size_t i) const { return m_views[i]; }
  std::span<const MeshView> views() const { return m_views; }
  size_t numViews() const { return m_views.size(); }

  /// \brief Add a stored MeshView object. The added view will be drawn when
  /// calling Renderer::draw() with a Mesh argument.
  /// \returns Reference to the created MeshView object.
  MeshView &addView(Uint32 vertexOffset, Uint32 vertexSubCount,
                    Uint32 indexOffset, Uint32 indexSubCount);

  /// \brief Bind an existing vertex buffer to a given slot of the Mesh.
  /// \warning This function will **take ownership of the buffer**.
  ///
  /// \param slot Binding slot of the vertex buffer. Used by
  /// SDL_GPUVertexAttribute.
  /// \param buffer Existing buffer.
  /// \returns Reference to \c this, for method chaining.
  Mesh &bindVertexBuffer(Uint32 slot, SDL_GPUBuffer *buffer);

  /// \brief Bind an existing index buffer for the Mesh.
  ///
  /// This function does **not** check that the buffer is non-null. The lack of
  /// check is to allow method chaining with arguments which may or may not be
  /// null, depending on context.
  /// \warning This function will **take ownership of the buffer**.
  ///
  /// \param buffer Existing index buffer.
  /// \returns Reference to \c this, for method chaining.
  Mesh &setIndexBuffer(SDL_GPUBuffer *buffer);

  /// \brief Number of vertex buffers.
  Uint32 numVertexBuffers() const {
    return static_cast<Uint32>(vertexBuffers.size());
  }

  bool isIndexed() const { return indexBuffer != nullptr; }

  /// \brief Release all owned vertex and index buffers in the Mesh object.
  void release() noexcept;
  ~Mesh() noexcept { release(); }

  SDL_GPUBufferBinding getVertexBinding(Uint32 slot) const {
    return {.buffer = vertexBuffers[slot], .offset = 0u};
  }

  SDL_GPUBufferBinding getIndexBinding() const {
    return {.buffer = indexBuffer, .offset = 0u};
  }
};

/// \brief Check that all vertex buffers were set, and consistency in the
/// "indexed/non-indexed" status.
/// \sa validateMeshView()
[[nodiscard]] inline bool validateMesh(const Mesh &mesh) {
  if (!validateMeshLayout(mesh.layout()))
    return false;
  if (mesh.views().size() == 0)
    return false;
  for (size_t i = 0; i < mesh.numVertexBuffers(); i++) {
    if (!mesh.vertexBuffers[i])
      return false;
  }
  // check that if we are indexed iff we have positive index count.
  return mesh.isIndexed() == (mesh.indexCount > 0);
}

/// \brief Validation for a MeshView object.
///
/// Check that all vertex buffer handles are non-null, check that vertex count
/// is nonzero, check consistency of indexing status (that index buffer is
/// non-null iff the index count of this view is nonzero).
/// \param view Input view to validate.
/// \sa validateMesh()
[[nodiscard]] inline bool validateMeshView(const MeshView &view) {
  for (auto vb : view.vertexBuffers) {
    if (!vb)
      return false;
  }
  // views of indexed meshes cannot have zero indices.
  if (view.isIndexed() != (view.indexCount > 0))
    return false;
  return view.vertexCount > 0;
}

} // namespace candlewick
