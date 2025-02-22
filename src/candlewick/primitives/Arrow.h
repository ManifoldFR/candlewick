#pragma once

#include "../utils/MeshData.h"

namespace candlewick {

/// \brief Load a solid 3D arrow.
/// \ingroup primitives1
MeshData loadArrowSolid(bool include_normals = false, float shaft_length = 0.4f,
                        float shaft_radius = 0.01f, float head_length = 0.1f,
                        float head_radius = 0.02f, Uint32 segments = 32);

/// \brief Create a 3D triad
/// \ingroup primitives1
std::array<MeshData, 3> loadTriadSolid(float shaft_length = 0.4f,
                                       float shaft_radius = 0.01f,
                                       float head_length = 0.1f,
                                       float head_radius = 0.02f,
                                       Uint32 segments = 32);

} // namespace candlewick
