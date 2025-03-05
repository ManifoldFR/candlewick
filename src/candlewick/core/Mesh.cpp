#include "Mesh.h"

#include "Device.h"
#include "MeshLayout.h"
#include "./errors.h"

#include <cassert>

namespace candlewick {

MeshView::MeshView(const MeshView &parent, Uint32 subVertexOffset,
                   Uint32 subVertexCount, Uint32 subIndexOffset,
                   Uint32 subIndexCount)
    : vertexBuffers(parent.vertexBuffers), indexBuffer(parent.indexBuffer),
      vertexOffset(parent.vertexOffset + subVertexOffset),
      vertexCount(subVertexCount),
      indexOffset(parent.indexOffset + subIndexOffset),
      indexCount(subIndexCount) {
  // assumption: parent MeshView is validated
  assert(validateMeshView(*this));
  assert(subVertexOffset + subVertexCount <= parent.vertexCount);
  assert(subIndexOffset + subIndexCount <= parent.indexCount);
}

Mesh::Mesh(NoInitT) : layoutHandle() {}

Mesh::Mesh(const Device &device, const entt::handle &layout)
    : m_device(device), layoutHandle(layout) {
  const Uint32 count = this->layout().numBuffers();
  vertexBuffers.resize(count, nullptr);
}

Mesh::Mesh(Mesh &&other) noexcept
    : m_device(other.m_device), m_views(std::move(other.m_views)),
      layoutHandle(other.layoutHandle), vertexCount(other.vertexCount),
      indexCount(other.indexCount),
      vertexBuffers(std::move(other.vertexBuffers)),
      indexBuffer(other.indexBuffer) {
  other.m_device = nullptr;
  other.indexBuffer = nullptr;
}

Mesh &Mesh::operator=(Mesh &&other) noexcept {
  if (this != &other) {
    if (m_device) {
      release();
    }

    m_device = other.m_device;
    m_views = std::move(other.m_views);
    layoutHandle = std::move(other.layoutHandle);
    vertexCount = std::move(other.vertexCount);
    indexCount = std::move(other.indexCount);
    vertexBuffers = std::move(other.vertexBuffers);
    indexBuffer = std::move(other.indexBuffer);

    other.m_device = nullptr;
    other.vertexCount = 0u;
    other.indexCount = 0u;
    other.indexBuffer = nullptr;
  }
  return *this;
}

Mesh &Mesh::bindVertexBuffer(Uint32 slot, SDL_GPUBuffer *buffer) {
  for (std::size_t i = 0; i < numVertexBuffers(); i++) {
    if (layout().m_bufferDescs[i].slot == slot) {
      if (vertexBuffers[i]) {
        throw InvalidArgument(
            "Rebinding vertex buffer to already occupied slot.");
      }
      vertexBuffers[i] = buffer;
      return *this;
    }
  }
  CDW_UNREACHABLE_ASSERT("Binding slot not found!");
  std::terminate();
}

Mesh &Mesh::setIndexBuffer(SDL_GPUBuffer *buffer) {
  indexBuffer = buffer;
  return *this;
}

void Mesh::release() noexcept {
  if (!m_device)
    return;

  for (std::size_t i = 0; i < vertexBuffers.size(); i++) {
    if (vertexBuffers[i])
      SDL_ReleaseGPUBuffer(m_device, vertexBuffers[i]);
  }
  vertexBuffers.clear();

  if (isIndexed()) {
    SDL_ReleaseGPUBuffer(m_device, indexBuffer);
    indexBuffer = nullptr;
  }
}

MeshView &Mesh::addView(Uint32 vertexOffset, Uint32 vertexSubCount,
                        Uint32 indexOffset, Uint32 indexSubCount) {
  MeshView v;
  v.vertexBuffers = vertexBuffers;
  v.indexBuffer = indexBuffer;
  v.vertexOffset = vertexOffset;
  v.vertexCount = vertexSubCount;
  v.indexOffset = indexOffset;
  v.indexCount = indexSubCount;

  return m_views.emplace_back(std::move(v));
}

} // namespace candlewick
