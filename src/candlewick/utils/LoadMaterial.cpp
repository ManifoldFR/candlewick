#include "LoadMaterial.h"
#include "candlewick/core/errors.h"

#include <magic_enum/magic_enum.hpp>

namespace candlewick {

std::string_view assimpPropertyTypeName(aiPropertyTypeInfo type) {
  return magic_enum::enum_name(type);
}

std::string_view assimpShadingModeName(aiShadingMode shading_mode) {
  return magic_enum::enum_name(shading_mode);
}

#define _col(name) (aiColor4D &)*name.data()

enum class material_load_retc {
  INVALID,
  PHONG_MISSING_DIFFUSE,
  PBR_MISSING_BASE_COLOR,
  PBR_MISSING_METALNESS,
  PBR_MISSING_ROUGHNESS,
  OK,
};

material_load_retc loadPhongMaterial(aiMaterial *material, PhongMaterial &out) {
  if (!material)
    return material_load_retc::INVALID;

  if (material->Get(AI_MATKEY_COLOR_DIFFUSE, _col(out.diffuse)) !=
      aiReturn_SUCCESS) {
    return material_load_retc::PHONG_MISSING_DIFFUSE;
  }
  material->Get(AI_MATKEY_COLOR_AMBIENT, _col(out.ambient));
  material->Get(AI_MATKEY_COLOR_SPECULAR, _col(out.specular));
  material->Get(AI_MATKEY_COLOR_EMISSIVE, _col(out.emissive));
  material->Get(AI_MATKEY_SHININESS, out.shininess);
  material->Get(AI_MATKEY_REFLECTIVITY, out.reflectivity);
  return material_load_retc::OK;
}

material_load_retc loadPbrMaterial(aiMaterial *material, PbrMaterial &out) {
  if (!material)
    return material_load_retc::INVALID;

  if (material->Get(AI_MATKEY_BASE_COLOR, _col(out.baseColor)) !=
      aiReturn_SUCCESS)
    return material_load_retc::PBR_MISSING_BASE_COLOR;

  material->Get(AI_MATKEY_METALLIC_FACTOR, out.metalness);
  material->Get(AI_MATKEY_ROUGHNESS_FACTOR, out.roughness);
  return material_load_retc::OK;
}

#undef _col

PbrMaterial pbrFromPhong(const PhongMaterial &phong) {
  PbrMaterial out;
  out.baseColor = phong.diffuse;
  auto specularRgb = phong.specular.head<3>();
  float specAvg = specularRgb.mean();
  float specVar = (specularRgb - Float3::Constant(specAvg)).squaredNorm();

  const float METALLIC_THRESHOLD = 0.2f;
  if (specVar > METALLIC_THRESHOLD) {
    out.metalness = .5f;
    out.baseColor = phong.specular;
  } else {
    out.metalness = 0.f;
    out.baseColor *= (1.0f + specAvg);
  }

  float shininess = std::clamp(phong.shininess, 1.0f, 1e3f);
  out.roughness = sqrtf(2.f / (shininess + 2.f));
  out.ao = phong.ambient.head<3>().mean();
  out.ao = fmaxf(0.1f, out.ao);
  return out;
}

PbrMaterial loadFromAssimpMaterial(aiMaterial *material) {
  PbrMaterial out;
  auto retc = loadPbrMaterial(material, out);
  if (retc == material_load_retc::OK)
    return out;

  PhongMaterial phong;
  retc = loadPhongMaterial(material, phong);

  if (retc == material_load_retc::OK)
    return pbrFromPhong(phong);

  terminate_with_message("Failed to load material: {:s}",
                         magic_enum::enum_name(retc));
}
std::string getBaseColorTexturePath(aiMaterial *material) {
  if (!material)
    return {};

  aiString texPath;
  // Try PBR base color first, then fall back to diffuse
  if (material->GetTexture(aiTextureType_BASE_COLOR, 0, &texPath) ==
      aiReturn_SUCCESS) {
    return std::string(texPath.C_Str());
  }
  if (material->GetTexture(aiTextureType_DIFFUSE, 0, &texPath) ==
      aiReturn_SUCCESS) {
    return std::string(texPath.C_Str());
  }
  return {};
}

} // namespace candlewick
