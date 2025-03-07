#include "MeshData.h"
#include "../core/Device.h"
#include "../core/Mesh.h"
#include "../core/CommandBuffer.h"

#include <SDL3/SDL_log.h>

namespace candlewick {

MeshData::MeshData(NoInitT) {}

MeshData::MeshData(SDL_GPUPrimitiveType primitiveType, const MeshLayout &layout,
                   std::vector<char> vertexData,
                   std::vector<IndexType> indexData)
    : m_vertexData(std::move(vertexData)), primitiveType(primitiveType),
      layout(layout), indexData(std::move(indexData)) {
  m_vertexSize = this->layout.vertexSize();
  m_numVertices = static_cast<Uint32>(m_vertexData.size()) / m_vertexSize;
}

Mesh createMesh(const Device &device, const MeshData &meshData, bool upload) {
  auto &layout = meshData.layout;
  SDL_GPUBufferCreateInfo vtxInfo{.usage = SDL_GPU_BUFFERUSAGE_VERTEX,
                                  .size = meshData.numVertices() *
                                          layout.vertexSize(),
                                  .props = 0};

  SDL_GPUBuffer *vertexBuffer = SDL_CreateGPUBuffer(device, &vtxInfo);
  SDL_GPUBuffer *indexBuffer = NULL;
  if (meshData.isIndexed()) {
    SDL_GPUBufferCreateInfo indexInfo{.usage = SDL_GPU_BUFFERUSAGE_INDEX,
                                      .size = meshData.numIndices() *
                                              layout.indexSize(),
                                      .props = 0};
    indexBuffer = SDL_CreateGPUBuffer(device, &indexInfo);
  }
  Mesh mesh = createMesh(device, meshData, vertexBuffer, indexBuffer);
  if (upload)
    uploadMeshToDevice(device, mesh, meshData);
  return mesh;
}

Mesh createMesh(const Device &device, const MeshData &meshData,
                SDL_GPUBuffer *vertexBuffer, SDL_GPUBuffer *indexBuffer) {
  Mesh mesh{device, meshData.layout};

  mesh.bindVertexBuffer(0, vertexBuffer);
  mesh.vertexCount = meshData.numVertices();
  mesh.indexCount = meshData.numIndices();
  if (meshData.isIndexed()) {
    mesh.setIndexBuffer(indexBuffer);
  }
  mesh.addView(0u, mesh.vertexCount, 0u, mesh.indexCount);
  return mesh;
}

Mesh createMeshFromBatch(const Device &device,
                         std::span<const MeshData> meshDatas, bool upload) {
  // index type size, in bytes
  assert(meshDatas.size() > 0);
  auto &layout = meshDatas[0].layout;

  Uint32 numVertices = 0, numIndices = 0;
  for (auto &data : meshDatas) {
    numVertices += data.numVertices();
    numIndices += data.numIndices();
  }
  Mesh mesh{device, meshDatas[0].layout};
  assert(mesh.numVertexBuffers() == 1);

  SDL_GPUBufferCreateInfo vtxInfo{.usage = SDL_GPU_BUFFERUSAGE_VERTEX,
                                  .size =
                                      Uint32(layout.vertexSize() * numVertices),
                                  .props = 0};

  SDL_GPUBufferCreateInfo idxInfo;

  if (numIndices > 0) {
    idxInfo = {.usage = SDL_GPU_BUFFERUSAGE_INDEX,
               .size = numIndices * layout.indexSize(),
               .props = 0};
  }

  auto masterVertexBuffer = SDL_CreateGPUBuffer(device, &vtxInfo);
  auto masterIndexBuffer =
      (numIndices > 0) ? SDL_CreateGPUBuffer(device, &idxInfo) : NULL;
  mesh.vertexCount = numVertices;
  mesh.indexCount = numIndices;
  mesh.bindVertexBuffer(0, masterVertexBuffer)
      .setIndexBuffer(masterIndexBuffer);

  Uint32 vertexOffset = 0, indexOffset = 0;
  for (size_t i = 0; i < meshDatas.size(); i++) {
    auto &view = mesh.addView(vertexOffset, meshDatas[i].numVertices(),
                              indexOffset, meshDatas[i].numIndices());
    vertexOffset += meshDatas[i].numVertices();
    indexOffset += meshDatas[i].numIndices();
    if (upload)
      uploadMeshToDevice(device, view, meshDatas[i]);
  }
  return mesh;
}

void uploadMeshToDevice(const Device &device, const MeshView &meshView,
                        const MeshData &meshData) {

  SDL_GPUTransferBuffer *transfer_buffer;
  SDL_GPUCopyPass *copy_pass;

  CommandBuffer upload_command_buffer{device};
  copy_pass = SDL_BeginGPUCopyPass(upload_command_buffer);

  auto &layout = meshData.layout;
  const Uint32 vertex_payload_size =
      meshData.numVertices() * layout.vertexSize();
  const Uint32 index_payload_size = meshData.numIndices() * layout.indexSize();
  const Uint32 total_payload_size = vertex_payload_size + index_payload_size;

  SDL_GPUTransferBufferCreateInfo transfer_buffer_desc{
      .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
      .size = total_payload_size,
      .props = 0,
  };
  transfer_buffer = SDL_CreateGPUTransferBuffer(device, &transfer_buffer_desc);
  {
    std::byte *map =
        (std::byte *)SDL_MapGPUTransferBuffer(device, transfer_buffer, false);
    // copy vertices
    {
      SDL_memcpy(map, meshData.data(), vertex_payload_size);
      SDL_GPUTransferBufferLocation src_location{
          .transfer_buffer = transfer_buffer,
          .offset = 0,
      };
      SDL_GPUBufferRegion dst_region{
          .buffer = meshView.vertexBuffers[0],
          .offset = meshView.vertexOffset * layout.vertexSize(),
          .size = vertex_payload_size,
      };
      SDL_UploadToGPUBuffer(copy_pass, &src_location, &dst_region, false);
    }
    // copy indices
    if (meshView.isIndexed()) {
      SDL_memcpy(map + vertex_payload_size, meshData.indexData.data(),
                 index_payload_size);
      SDL_GPUTransferBufferLocation src_location{
          .transfer_buffer = transfer_buffer,
          .offset = vertex_payload_size,
      };
      SDL_GPUBufferRegion dst_region{
          .buffer = meshView.indexBuffer,
          .offset = meshView.indexOffset * layout.indexSize(),
          .size = index_payload_size,
      };
      SDL_UploadToGPUBuffer(copy_pass, &src_location, &dst_region, false);
    }
    SDL_UnmapGPUTransferBuffer(device, transfer_buffer);
  }
  SDL_ReleaseGPUTransferBuffer(device, transfer_buffer);
  SDL_EndGPUCopyPass(copy_pass);
  if (!SDL_SubmitGPUCommandBuffer(upload_command_buffer)) {
    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                 "%s: Failed to submit command buffer: %s", __FILE__,
                 SDL_GetError());
    assert(false);
  }
}

void uploadMeshToDevice(const Device &device, const Mesh &mesh,
                        const MeshData &meshData) {
  assert(validateMesh(mesh));
  assert(mesh.numViews() == 1);
  uploadMeshToDevice(device, mesh.view(0), meshData);
}

} // namespace candlewick
