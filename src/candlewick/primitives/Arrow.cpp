#include "Arrow.h"
#include "Internal.h"
#include "../utils/MeshTransforms.h"

namespace candlewick {

namespace detail {

  static ConeCylinderBuilder
  gen_arrow_impl(float shaft_length, float shaft_radius, float head_length,
                 float head_radius, Uint32 segments) {
    ConeCylinderBuilder mesh;

    // Generate shaft (cylinder)
    const float angleIncrement = 2.f * constants::Pif / float(segments);

    // Shaft vertices
    for (Uint32 i = 0; i <= segments; ++i) {
      float angle = float(i) * angleIncrement;
      float x = std::cos(angle);
      float y = std::sin(angle);

      // Bottom circle
      mesh.positions.push_back({x * shaft_radius, y * shaft_radius, 0});
      mesh.normals.push_back({x, y, 0});

      // Top circle
      mesh.positions.push_back(
          {x * shaft_radius, y * shaft_radius, shaft_length});
      mesh.normals.push_back({x, y, 0});
    }

    // Shaft indices (triangle strip)
    for (Uint32 i = 0; i < segments * 2; i += 2) {
      mesh.addFace({i, i + 1, i + 2});
      mesh.addFace({i + 1, i + 3, i + 2});
    }

    // Generate head (cone)
    mesh.addCone(segments, head_radius, shaft_length, head_length);

    return mesh;
  }
} // namespace detail

std::vector<char> interleaveAttributes(const detail::ConeCylinderBuilder &arrow,
                                       bool include_normals) {
  Uint32 stride = include_normals ? 32u : 16u;
  Uint64 vertexCount = arrow.positions.size();
  std::vector<char> vertex_data(vertexCount * stride); // 32 byte stride

  for (size_t i = 0; i < arrow.positions.size(); i++) {
    char *vertex = vertex_data.data() + (i * stride); // Point to current vertex

    // Position (12 bytes + 4 padding)
    memcpy(vertex, &arrow.positions[i], sizeof(Float3));
    // Skip 4 bytes padding

    if (include_normals) {
      // Normal (12 bytes + 4 padding)
      memcpy(vertex + 16, &arrow.normals[i], sizeof(Float3));
      // Final 4 bytes padding
    }
  }
  return vertex_data;
}

MeshData loadArrowSolid(bool include_normals, float shaft_length,
                        float shaft_radius, float head_length,
                        float head_radius, Uint32 segments) {
  auto arrow_data = detail::gen_arrow_impl(shaft_length, shaft_radius,
                                           head_length, head_radius, segments);
  std::vector<char> vertexData =
      interleaveAttributes(arrow_data, include_normals);
  MeshLayout layout;
  layout.addBinding(0, include_normals ? 32u : 16u);
  layout.addAttribute(VertexAttrib::Position, 0,
                      SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, 0);
  if (include_normals)
    layout.addAttribute(VertexAttrib::Normal, 0,
                        SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, 16u);
  return MeshData{SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, layout,
                  std::move(vertexData), std::move(arrow_data.indices)};
}

std::array<MeshData, 3> loadTriadSolid(float shaft_length, float shaft_radius,
                                       float head_length, float head_radius,
                                       Uint32 segments) {
  MeshData zAxis = loadArrowSolid(false, shaft_length, shaft_radius,
                                  head_length, head_radius, segments);
  MeshData xAxis = MeshData::copy(zAxis);
  MeshData yAxis = MeshData::copy(zAxis);
  xAxis.material.baseColor << 1., 0., 0., 1.;
  yAxis.material.baseColor << 0., 1., 0., 1.;
  zAxis.material.baseColor << 0., 0., 1., 1.;
  const Eigen::AngleAxisf aax{constants::Pi_2f, Float3::UnitY()};
  const Eigen::AngleAxisf aay{constants::Pi_2f, -Float3::UnitX()};
  apply3DTransformInPlace(xAxis, Eigen::Affine3f{aax});
  apply3DTransformInPlace(yAxis, Eigen::Affine3f{aay});
  return {std::move(xAxis), std::move(yAxis), std::move(zAxis)};
}

} // namespace candlewick
