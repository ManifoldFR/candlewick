#include "MeshData.h"
#include "MeshTransforms.h"

#include <SDL3/SDL_assert.h>
#include <numeric>

namespace candlewick {

void apply3DTransformInPlace(MeshData &meshData, const Eigen::Affine3f &tr) {

  const MeshLayout &layout = meshData.layout;

  if (auto posAttr = layout.getAttribute(VertexAttrib::Position)) {
    for (Uint64 i = 0; i < meshData.numVertices(); i++) {
      Float3 &pos = meshData.getAttribute<Float3>(i, *posAttr);
      pos = tr * pos;
    }
  }

  Eigen::Matrix3f normalMatrix = tr.linear().inverse().transpose();
  if (auto normAttr = layout.getAttribute(VertexAttrib::Normal)) {
    for (Uint64 i = 0; i < meshData.numVertices(); i++) {
      Float3 &normal = meshData.getAttribute<Float3>(i, *normAttr);
      normal.applyOnTheLeft(normalMatrix);
    }
  }

  if (auto tangAttr = layout.getAttribute(VertexAttrib::Tangent)) {
    for (Uint64 i = 0; i < meshData.numVertices(); i++) {
      Float3 &tang = meshData.getAttribute<Float3>(i, *tangAttr);
      tang.applyOnTheLeft(normalMatrix);
    }
  }
}

void triangleStripGenerateIndices(Uint32 vertexCount,
                                  std::vector<Uint32> &indices) {
  const Uint32 iMax = std::max(vertexCount, 2u) - 2u;
  indices.resize(3 * iMax);
  for (Uint32 i = 0; i != iMax; i++) {
    indices[3 * i + 0] = i % 2 ? i + 1 : i;
    indices[3 * i + 1] = i % 2 ? i : i + 1;
    indices[3 * i + 2] = i + 2;
  }
}

MeshData generateIndices(const MeshData &meshData) {
  SDL_assert(meshData.primitiveType == SDL_GPU_PRIMITIVETYPE_TRIANGLESTRIP);
  SDL_assert(!meshData.isIndexed());
  std::vector<Uint32> indices;
  Uint32 vertexCount = Uint32(meshData.numVertices());
  triangleStripGenerateIndices(vertexCount, indices);
  std::vector<char> vertexData{meshData.data(),
                               meshData.data() + meshData.vertexBytes()};
  return MeshData{SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, meshData.layout,
                  std::move(vertexData), std::move(indices)};
}

namespace detail {
  std::pair<Uint32, Uint32>
  mergeCalcIndexVertexCount(std::span<const MeshData> meshes) {
    Uint32 indexCount = 0, vertexCount = 0;
    for (const MeshData &m : meshes) {
      // if no indices, use vertex count.
      // if the mesh is indexed, use the index count.
      // if moreover this is the first indexed mesh,
      // then use the vertex count from all previous meshes.
      if (m.isIndexed()) {
        if (!indexCount)
          indexCount = vertexCount;
        indexCount += m.numIndices();
      } else if (indexCount) {
        indexCount += m.numVertices();
      }
      vertexCount += m.numVertices();
    }
    return {indexCount, vertexCount};
  }
} // namespace detail

MeshData mergeMeshes(std::span<const MeshData> meshes) {
  SDL_assert(!meshes.empty());
  // 1. check coherence of primitives
  SDL_GPUPrimitiveType primitive = meshes[0].primitiveType;
  auto layout = meshes[0].layout;
  for (const MeshData &m : meshes) {
    SDL_assert(m.primitiveType == primitive);
  }
  auto [indexCount, vertexCount] = detail::mergeCalcIndexVertexCount(meshes);
  std::vector<char> vertexData;
  vertexData.resize(vertexCount * layout.vertexSize());
  std::vector<MeshData::IndexType> indexData;
  indexData.resize(indexCount);

  Uint32 vtxOffset = 0, indexOffset = 0;
  // three cases:
  // 1. indexed: copy indices over to our indexData vector, shift them by the
  // number of vertices
  // 2. not indexed but we are generating an indexed output mesh: generate
  // trivial indices
  // 3. neither: do nothing, just increment
  for (size_t i = 0; i < meshes.size(); i++) {
    const MeshData &mesh = meshes[i];
    if (mesh.isIndexed()) {
      std::span<MeshData::IndexType> dst{indexData.begin() + indexOffset,
                                         mesh.numIndices()};
      std::copy_n(mesh.indexData.begin(), mesh.numIndices(), dst.begin());
      indexOffset += mesh.numIndices();

      for (Uint32 &index : dst)
        index += vtxOffset;
    } else if (!indexData.empty()) {
      std::iota(indexData.begin() + indexOffset,
                indexData.begin() + indexOffset + Uint32(mesh.numIndices()),
                Uint32(vtxOffset));
    }

    Uint32 numVtxBytes = mesh.vertexSize() * mesh.numVertices();
    std::copy_n(mesh.data(), numVtxBytes,
                vertexData.begin() + vtxOffset * layout.vertexSize());

    vtxOffset += mesh.numVertices();
  }

  return MeshData{primitive, meshes[0].layout, std::move(vertexData),
                  std::move(indexData)};
}

MeshData mergeMeshes(std::vector<MeshData> &&meshes) {
  std::span<const MeshData> view{meshes};
  return mergeMeshes(view);
}

} // namespace candlewick
