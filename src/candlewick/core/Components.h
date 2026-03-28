#pragma once
#include "math_types.h"
#include "Mesh.h"
#include "Texture.h"
#include "MaterialUniform.h"

#include <entt/entity/fwd.hpp>

namespace candlewick {

/// Tag struct for denoting an entity as opaque, for render pass organization.
struct Opaque {};

/// Tag struct for disabled (invisible) entities.
struct Disable {};

// Tag environment entities
struct EnvironmentTag {};

struct TransformComponent : Mat4f {
  using Mat4f::Mat4f;
  using Mat4f::operator=;
};

enum class RenderMode { FILL, LINE };

struct MeshMaterialComponent {
  Mesh mesh;
  std::vector<PbrMaterial> materials;
  std::vector<Texture> baseColorTextures; //< One per mesh view (use white
                                          //  fallback when no texture).
  RenderMode mode = RenderMode::FILL;
  MeshMaterialComponent(Mesh &&mesh, std::vector<PbrMaterial> &&materials)
      : mesh(std::move(mesh)), materials(std::move(materials)) {
    assert(mesh.numViews() == materials.size());
  }

  MeshMaterialComponent(Mesh &&mesh, std::vector<PbrMaterial> &&materials,
                         std::vector<Texture> &&textures)
      : mesh(std::move(mesh)), materials(std::move(materials)),
        baseColorTextures(std::move(textures)) {
    assert(mesh.numViews() == materials.size());
  }

  bool hasTransparency() const;
};

/// \brief Updates (adds or removes) the Opaque tag component for a given
/// entity.
/// \param reg The entity registry
/// \param entity The entity
/// \param mmc The mesh-material element to inspect for existence of any
/// transparent subobjects.
/// \returns whether the entity is transparent.
bool updateTransparencyClassification(entt::registry &reg, entt::entity entity,
                                      const MeshMaterialComponent &mmc);

inline void toggleDisable(entt::registry &reg, entt::entity id, bool flag) {
  if (flag)
    reg.remove<Disable>(id);
  else
    reg.emplace<Disable>(id);
}

} // namespace candlewick
