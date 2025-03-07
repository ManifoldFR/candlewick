#include "MeshDataView.h"

namespace candlewick {

MeshDataView::MeshDataView(const MeshData &meshData)
    : primitiveType(meshData.primitiveType), layout(meshData.layout),
      vertexData(meshData.data(), meshData.numVertices() * layout.vertexSize()),
      indexData(meshData.indexData) {}

MeshDataView::MeshDataView(SDL_GPUPrimitiveType primitiveType,
                           MeshLayout layout, std::span<const char> vertices,
                           std::span<const IndexType> indices)
    : primitiveType(primitiveType), layout(layout), vertexData(vertices),
      indexData(indices) {}

MeshData toOwningMeshData(const MeshDataView &view) {
  std::vector<char> vertexData;
  vertexData.assign(view.vertexData.begin(), view.vertexData.end());
  std::vector<MeshData::IndexType> indexData;
  indexData.assign(view.indexData.begin(), view.indexData.end());
  return MeshData{view.primitiveType, view.layout, std::move(vertexData),
                  std::move(indexData)};
}

} // namespace candlewick
