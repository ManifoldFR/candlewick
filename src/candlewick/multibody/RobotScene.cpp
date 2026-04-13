#include "RobotScene.h"
#include "LoadPinocchioGeometry.h"
#include "../core/Components.h"
#include "../core/Shader.h"
#include "../core/Components.h"
#include "../core/TransformUniforms.h"
#include "../core/Camera.h"
#include "../utils/LoadTexture.h"

#include <entt/entity/registry.hpp>
#include <coal/BVH/BVH_model.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <ranges>
#include <magic_enum/magic_enum_utility.hpp>
#include <magic_enum/magic_enum_switch.hpp>

namespace candlewick {
/// \brief Terminate the application after encountering an invalid enum value.
template <typename T>
  requires std::is_enum_v<T>
[[noreturn]] void
invalid_enum(const char *msg, T type,
             std::source_location location = std::source_location::current()) {
  terminate_with_message(location, "Invalid enum: {:s} - {:s}", msg,
                         magic_enum::enum_name(type));
}
} // namespace candlewick

namespace candlewick::multibody {

struct alignas(16) LightArrayUbo {
  GpuVec4 viewSpaceDir[kNumLights];
  GpuVec4 color[kNumLights];
  GpuVec4 intensity[kNumLights];
  Uint32 numLights;
  Uint32 useSsao;
};
static_assert(std::is_standard_layout_v<LightArrayUbo>);

struct alignas(16) LightSpaceMatricesUbo {
  GpuMat4 mvps[kNumLights];
  Uint32 numLights;
};

struct alignas(16) ShadowAtlasInfoUbo {
  using Vec4u = Eigen::Matrix<Uint32, 4, 1, Eigen::DontAlign>;
  std::array<Vec4u, kNumLights> regions;
};

void updateRobotTransforms(entt::registry &registry,
                           const pin::GeometryModel &geom_model,
                           const pin::GeometryData &geom_data) {
  auto view = registry.view<const PinGeomObjComponent, TransformComponent,
                            MeshMaterialComponent>();
  for (auto [ent, geom_id, tr, mmc] : view.each()) {
    auto &gobj = geom_model.geometryObjects[geom_id];
    SE3f pose = geom_data.oMg[geom_id].cast<float>();
    Float3 scale = gobj.meshScale.cast<float>();
    Float4 color = gobj.meshColor.cast<float>();
    auto D = scale.homogeneous().asDiagonal();
    tr.noalias() = pose.toHomogeneousMatrix() * D;
    if (gobj.overrideMaterial) {
      for (auto &mat : mmc.materials)
        mat.baseColor = color;
      if (color.w() < 1.0f) {
        registry.remove<Opaque>(ent);
      } else {
        registry.emplace_or_replace<Opaque>(ent);
      }
    }
  }
}

auto RobotScene::pinGeomToPipeline(const coal::CollisionGeometry &geom)
    -> PipelineType {
  const auto objType = geom.getObjectType();
  switch (objType) {
    using enum coal::OBJECT_TYPE;
  case OT_GEOM:
    return PIPELINE_TRIANGLEMESH;
  case OT_HFIELD:
    return PIPELINE_HEIGHTFIELD;
  case OT_BVH: {
    using enum coal::BVHModelType;
    // cast to BVHModelBase, check if mesh or points
    const auto &bvh = dynamic_cast<const coal::BVHModelBase &>(geom);
    switch (bvh.getModelType()) {
    case BVH_MODEL_POINTCLOUD:
      return PIPELINE_POINTCLOUD;
    case BVH_MODEL_TRIANGLES:
      return PIPELINE_TRIANGLEMESH;
    case BVH_MODEL_UNKNOWN:
      invalid_enum("Unknown BVH model type.", BVH_MODEL_UNKNOWN);
    }
  }
  case OT_COUNT:
  case OT_OCTREE:
  case OT_UNKNOWN:
    invalid_enum("Unsupported object type", objType);
  }
}

static void addPipelineTagComponent(entt::registry &reg, entt::entity ent,
                                    RobotScene::PipelineType type) {
  magic_enum::enum_switch(
      [&reg, ent](auto pt) { reg.emplace<RobotScene::pipeline_tag<pt>>(ent); },
      type);
}

entt::entity RobotScene::addEnvironmentObject(MeshData &&data, Mat4f placement,
                                              PipelineType pipe_type) {
  Mesh mesh = createMesh(device(), data, true);

  std::vector<Texture> textures;
  if (!data.baseColorTexturePath.empty()) {
    textures.push_back(
        loadTextureFromFile(device(), data.baseColorTexturePath.c_str()));
  } else {
    textures.push_back(createWhiteTexture(device()));
  }

  entt::entity entity = m_registry.create();
  m_registry.emplace<TransformComponent>(entity, placement);
  if (pipe_type != PIPELINE_POINTCLOUD)
    m_registry.emplace<Opaque>(entity);
  // add tag type
  m_registry.emplace<EnvironmentTag>(entity);
  const auto &mmc = m_registry.emplace<MeshMaterialComponent>(
      entity, std::move(mesh), std::vector{std::move(data.material)},
      std::move(textures));
  updateTransparencyClassification(m_registry, entity, mmc);
  addPipelineTagComponent(m_registry, entity, pipe_type);
  return entity;
}

void RobotScene::clearEnvironment() {
  auto view = m_registry.view<EnvironmentTag>();
  m_registry.destroy(view.begin(), view.end());
}

void RobotScene::clearRobotGeometries() {
  auto view = m_registry.view<PinGeomObjComponent>();
  m_registry.destroy(view.begin(), view.end());
}

RobotScene::RobotScene(entt::registry &registry, const RenderContext &renderer)
    : m_registry(registry)
    , m_renderer(renderer)
    , m_config()
    , m_geomModel(nullptr)
    , m_geomData(nullptr)
    , m_initialized(false)
    , m_pipelines()
    , m_whiteTexture(createWhiteTexture(renderer.device))
    , m_materialSampler(createMaterialSampler(renderer.device)) {
  assert(!hasInternalPointers());
  SDL_zero(directionalLight);
  directionalLight[1].direction = {-1.f, 1.f, -1.f};
  directionalLight[1].color.setOnes();
  directionalLight[1].intensity = 4.f;
}

RobotScene::RobotScene(entt::registry &registry, const RenderContext &renderer,
                       const pin::GeometryModel &geom_model,
                       const pin::GeometryData &geom_data, Config config)
    : RobotScene(registry, renderer) {

  this->setConfig(config);
  this->loadModels(geom_model, geom_data);
}

auto createTextureWithMultisampledVariant(const Device &device,
                                          SDL_GPUTextureCreateInfo texture_desc,
                                          const char *name) {
  const auto sample_count = texture_desc.sample_count;
  auto texture_desc_resolve = texture_desc;
  texture_desc_resolve.sample_count = SDL_GPU_SAMPLECOUNT_1;
  if (!SDL_GPUTextureSupportsSampleCount(device, texture_desc.format,
                                         sample_count)) {
    terminate_with_message(
        "Texture with format {:s} does not support sample count {:d}",
        magic_enum::enum_name(texture_desc.format),
        sdlSampleToValue(sample_count));
  }
  if (!SDL_GPUTextureSupportsFormat(device, texture_desc.format,
                                    texture_desc.type, texture_desc.usage)) {
    terminate_with_message("Texture format + type + usage unsupported.");
  }
  return std::tuple{
      Texture{device, texture_desc, name},
      Texture{device, texture_desc_resolve, name},
  };
}

void RobotScene::initGBuffer() {
  auto sample_count = m_renderer.getMsaaSampleCount();
  const auto [width, height] = m_renderer.window.sizeInPixels();
  std::tie(gBuffer.normalMap, gBuffer.resolveNormalMap) =
      createTextureWithMultisampledVariant(
          device(),
          {
              .type = SDL_GPU_TEXTURETYPE_2D,
              .format = SDL_GPU_TEXTUREFORMAT_R16G16_FLOAT,
              .usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET |
                       SDL_GPU_TEXTUREUSAGE_SAMPLER,
              .width = Uint32(width),
              .height = Uint32(height),
              .layer_count_or_depth = 1,
              .num_levels = 1,
              .sample_count = sample_count,
              .props = 0,
          },
          "GBuffer [Normal map]");

  std::tie(gBuffer.depthCopyTex, gBuffer.resolveDepthCopyTex) =
      createTextureWithMultisampledVariant(
          device(),
          {
              .type = SDL_GPU_TEXTURETYPE_2D,
              .format = SDL_GPU_TEXTUREFORMAT_R32_FLOAT,
              .usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET |
                       SDL_GPU_TEXTUREUSAGE_SAMPLER,
              .width = Uint32(width),
              .height = Uint32(height),
              .layer_count_or_depth = 1,
              .num_levels = 1,
              .sample_count = sample_count,
              .props = 0,
          },
          "GBuffer [Depth copy]");

  std::tie(gBuffer.accumTexture, gBuffer.resolveAccumTexture) =
      createTextureWithMultisampledVariant(
          device(),
          {
              .type = SDL_GPU_TEXTURETYPE_2D,
              .format = SDL_GPU_TEXTUREFORMAT_R16G16B16A16_FLOAT,
              .usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET |
                       SDL_GPU_TEXTUREUSAGE_SAMPLER,
              .width = Uint32(width),
              .height = Uint32(height),
              .layer_count_or_depth = 1,
              .num_levels = 1,
              .sample_count = sample_count,
              .props = 0,
          },
          "WBOIT Accumulation");

  std::tie(gBuffer.revealTexture, gBuffer.resolveRevealTexture) =
      createTextureWithMultisampledVariant(
          device(),
          {
              .type = SDL_GPU_TEXTURETYPE_2D,
              .format = SDL_GPU_TEXTUREFORMAT_R8_UNORM,
              .usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET |
                       SDL_GPU_TEXTUREUSAGE_SAMPLER,
              .width = Uint32(width),
              .height = Uint32(height),
              .layer_count_or_depth = 1,
              .num_levels = 1,
              .sample_count = sample_count,
              .props = 0,
          },
          "WBOIT Revealage");

  SDL_GPUSamplerCreateInfo sic{
      .min_filter = SDL_GPU_FILTER_LINEAR,
      .mag_filter = SDL_GPU_FILTER_LINEAR,
      .address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
      .address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
      .address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
  };
  gBuffer.sampler = SDL_CreateGPUSampler(device(), &sic);
}

void RobotScene::initCompositePipeline(const MeshLayout &layout) {
  Shader vertexShader = Shader::fromMetadata(device(), "DrawQuad.vert");
  Shader fragmentShader = Shader::fromMetadata(device(), "WBOITComposite.frag");

  SDL_GPUColorTargetDescription color_target;
  SDL_zero(color_target);
  color_target.format = m_renderer.getSwapchainTextureFormat();
  color_target.blend_state = {
      .src_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE,
      .dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
      .color_blend_op = SDL_GPU_BLENDOP_ADD,
      .src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE,
      .dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
      .alpha_blend_op = SDL_GPU_BLENDOP_ADD,
      .enable_blend = true,
  };

  SDL_GPUGraphicsPipelineCreateInfo desc{
      .vertex_shader = vertexShader,
      .fragment_shader = fragmentShader,
      .vertex_input_state = layout,
      .primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
      .rasterizer_state{
          .fill_mode = SDL_GPU_FILLMODE_FILL,
          .cull_mode = SDL_GPU_CULLMODE_NONE,
      },
      .multisample_state{m_renderer.getMsaaSampleCount()},
      .depth_stencil_state{
          .enable_depth_test = false,
          .enable_depth_write = false,
      },
      .target_info{
          .color_target_descriptions = &color_target,
          .num_color_targets = 1,
          .has_depth_stencil_target = false,
      },
  };

  m_wboitComposite = GraphicsPipeline(device(), desc, "wboitComposite");
}

void RobotScene::ensurePipelinesExist(
    const std::set<pipeline_req_t> &required_pipelines) {
  if (!gBuffer.initialized()) {
    this->initGBuffer();
  }

  const bool enable_shadows = m_config.enable_shadows;

  for (auto &[layout, key] : required_pipelines) {
    if (!m_pipelines.contains(key)) {
      auto pipeline = createRenderPipeline(
          key, layout, m_renderer.colorFormat(), m_renderer.depthFormat());
      m_pipelines.set(key, std::move(pipeline));
    }

    const bool has_msaa = m_renderer.msaaEnabled();
    // handle other pipelines for effects
    if (key.type == PIPELINE_TRIANGLEMESH) {
      if (!ssaoPass.pipeline.initialized()) {
        ssaoPass = ssao::SsaoPass(
            m_renderer, has_msaa ? gBuffer.resolveNormalMap : gBuffer.normalMap,
            has_msaa ? gBuffer.resolveDepthCopyTex : gBuffer.depthCopyTex,
            m_config.ssao_kernel_size);
      }
      // configure shadow pass
      if (enable_shadows && !shadowPass.initialized()) {
        shadowPass = ShadowMapPass(device(), layout, m_renderer.depthFormat(),
                                   m_config.shadow_config);
      }
      if (!m_wboitComposite.initialized())
        this->initCompositePipeline(layout);
    }
  }
}

void RobotScene::loadModels(const pin::GeometryModel &geom_model,
                            const pin::GeometryData &geom_data) {
  if (hasInternalPointers())
    this->clearRobotGeometries();

  m_geomModel = &geom_model;
  m_geomData = &geom_data;

  // Phase 1. Load robot geometries and collect parameters for creating the
  // required render pipelines.
  std::set<pipeline_req_t> required_pipelines;

  for (pin::GeomIndex geom_id = 0; geom_id < geom_model.ngeoms; geom_id++) {

    const auto &geom_obj = geom_model.geometryObjects[geom_id];
    auto meshDatas = loadGeometryObject(geom_obj);
    PipelineType pipeline_type = pinGeomToPipeline(*geom_obj.geometry);
    Mesh mesh = createMeshFromBatch(device(), meshDatas, true);
    assert(validateMesh(mesh));

    // Load base color textures for each sub-mesh
    std::vector<Texture> textures;
    textures.reserve(meshDatas.size());
    for (const auto &md : meshDatas) {
      if (!md.baseColorTexturePath.empty()) {
        textures.push_back(
            loadTextureFromFile(device(), md.baseColorTexturePath.c_str()));
      } else {
        // Use a copy-reference: we cannot copy Texture, so create a fresh white
        textures.push_back(createWhiteTexture(device()));
      }
    }

    // add entity for this geometry
    entt::entity entity = m_registry.create();
    m_registry.emplace<PinGeomObjComponent>(entity, geom_id);
    m_registry.emplace<TransformComponent>(entity);
    const MeshMaterialComponent &mmc =
        m_registry.emplace<MeshMaterialComponent>(entity, std::move(mesh),
                                                  extractMaterials(meshDatas),
                                                  std::move(textures));
    if (pipeline_type != PIPELINE_POINTCLOUD)
      m_registry.emplace<Opaque>(entity);
    bool is_transparent =
        updateTransparencyClassification(m_registry, entity, mmc);
    addPipelineTagComponent(m_registry, entity, pipeline_type);

    auto &layout = mmc.mesh.layout();
    required_pipelines.insert(
        {layout, {pipeline_type, is_transparent, RenderMode::FILL}});
    required_pipelines.insert(
        {layout, {pipeline_type, is_transparent, RenderMode::LINE}});
  }

  // Phase 2. Init our render pipelines.
  this->ensurePipelinesExist(required_pipelines);
  m_initialized = true;
}

void RobotScene::update() {
  updateRobotTransforms(registry(), geomModel(), geomData());
}

void RobotScene::collectOpaqueCastables() {
  auto all_view = m_registry.view<const Opaque, const TransformComponent,
                                  const MeshMaterialComponent,
                                  pipeline_tag<PIPELINE_TRIANGLEMESH>>(
      entt::exclude<Disable>);

  m_castables.clear();

  // collect castable objects
  all_view.each([this](auto &&tr, auto &&meshMaterial) {
    m_castables.emplace_back(meshMaterial.mesh, tr);
  });
}

void RobotScene::renderOpaque(CommandBuffer &command_buffer,
                              const Camera &camera) {
  if (m_config.enable_ssao) {
    ssaoPass.render(command_buffer, camera);
  }

  renderPBRTriangleGeometry(command_buffer, camera, false);
  renderOtherGeometry(command_buffer, camera);
}

void RobotScene::renderTransparent(CommandBuffer &command_buffer,
                                   const Camera &camera) {
  renderPBRTriangleGeometry(command_buffer, camera, true);
  compositeTransparencyPass(command_buffer);
}

void RobotScene::render(CommandBuffer &command_buffer, const Camera &camera) {
  renderOpaque(command_buffer, camera);
  renderTransparent(command_buffer, camera);
}

/// Function private to this translation unit.
/// Utility function to provide a render pass handle
/// with just two configuration options: whether to load or clear the color and
/// depth targets.
static SDL_GPURenderPass *
getOpaqueRenderPass(const RenderContext &renderer,
                    CommandBuffer &command_buffer, SDL_GPULoadOp color_load_op,
                    SDL_GPULoadOp depth_load_op, bool has_normals_target,
                    const RobotScene::GBuffer &gbuffer) {
  SDL_GPUColorTargetInfo color_targets[3];
  SDL_zero(color_targets);
  color_targets[0].texture = renderer.colorTarget();
  color_targets[0].load_op = color_load_op;
  color_targets[0].store_op = SDL_GPU_STOREOP_STORE;
  color_targets[0].cycle = false;

  SDL_GPUDepthStencilTargetInfo depth_target;
  SDL_zero(depth_target);
  depth_target.texture = renderer.depthTarget();
  depth_target.clear_depth = 1.0f;
  depth_target.load_op = depth_load_op;
  depth_target.store_op = SDL_GPU_STOREOP_STORE;
  depth_target.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
  depth_target.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
  if (has_normals_target) {
    color_targets[1].texture = gbuffer.normalMap;
    color_targets[1].load_op = SDL_GPU_LOADOP_CLEAR;
    color_targets[1].store_op = SDL_GPU_STOREOP_STORE;
    color_targets[1].cycle = false;
    if (renderer.msaaEnabled()) {
      color_targets[1].resolve_texture = gbuffer.resolveNormalMap;
      color_targets[1].store_op = SDL_GPU_STOREOP_RESOLVE;
    }
    color_targets[2].texture = gbuffer.depthCopyTex;
    color_targets[2].load_op = SDL_GPU_LOADOP_CLEAR;
    color_targets[2].store_op = SDL_GPU_STOREOP_STORE;
    color_targets[2].cycle = false;
    if (renderer.msaaEnabled()) {
      color_targets[2].resolve_texture = gbuffer.resolveDepthCopyTex;
      color_targets[2].store_op = SDL_GPU_STOREOP_RESOLVE;
    }
  }
  Uint32 num_color_targets = has_normals_target ? 3 : 1;
  return SDL_BeginGPURenderPass(command_buffer, color_targets,
                                num_color_targets, &depth_target);
}

static SDL_GPURenderPass *
getTransparentRenderPass(const RenderContext &renderer,
                         CommandBuffer &command_buffer,
                         const RobotScene::GBuffer &gBuffer) {
  SDL_GPUColorTargetInfo targets[2];
  SDL_zero(targets);
  targets[0].texture = gBuffer.accumTexture;
  targets[0].load_op = SDL_GPU_LOADOP_CLEAR;
  targets[0].store_op = SDL_GPU_STOREOP_STORE;

  targets[1].texture = gBuffer.revealTexture;
  targets[1].clear_color = {1, 0, 0, 0}; // clear to red, since it's R8
  targets[1].load_op = SDL_GPU_LOADOP_CLEAR;
  targets[1].store_op = SDL_GPU_STOREOP_STORE;

  SDL_GPUDepthStencilTargetInfo depth_target;
  SDL_zero(depth_target);
  depth_target.texture = renderer.depthTarget();
  depth_target.load_op = SDL_GPU_LOADOP_LOAD;
  depth_target.store_op = SDL_GPU_STOREOP_STORE;

  return SDL_BeginGPURenderPass(command_buffer, targets, 2, &depth_target);
}

void RobotScene::compositeTransparencyPass(CommandBuffer &command_buffer) {
  // transparent triangle pipeline required
  if (!m_wboitComposite.initialized())
    return;

  SDL_GPUColorTargetInfo target;
  SDL_zero(target);
  target.texture = m_renderer.colorTarget();
  // op is LOAD - we want to keep results from opaque pass
  target.load_op = SDL_GPU_LOADOP_LOAD;
  target.store_op = SDL_GPU_STOREOP_STORE;
  const bool has_msaa = m_renderer.msaaEnabled();
  if (has_msaa) {
    target.resolve_texture = m_renderer.resolvedColorTarget();
    target.store_op = SDL_GPU_STOREOP_RESOLVE;
  }

  SDL_GPURenderPass *render_pass =
      SDL_BeginGPURenderPass(command_buffer, &target, 1, nullptr);
  m_wboitComposite.bind(render_pass);

  // Bind accumulation and revealage textures
  rend::bindFragmentSamplers(
      render_pass, 0,
      {
          {.texture = gBuffer.accumTexture, .sampler = gBuffer.sampler},
          {.texture = gBuffer.revealTexture, .sampler = gBuffer.sampler},
      });

  // Draw fullscreen quad
  SDL_DrawGPUPrimitives(render_pass, 6, 1, 0, 0);

  SDL_EndGPURenderPass(render_pass);
}

void RobotScene::renderPBRTriangleGeometry(CommandBuffer &command_buffer,
                                           const Camera &camera,
                                           bool transparent) {

  const Uint32 numLights = shadowPass.numLights();
  // calculate light ubos
  LightArrayUbo lightUbo;
  lightUbo.numLights = numLights;
  lightUbo.useSsao = m_config.enable_ssao ? 1u : 0u;
  for (size_t i = 0; i < lightUbo.numLights; i++) {
    auto &dl = directionalLight[i];
    lightUbo.viewSpaceDir[i].head<3>() = camera.transformVector(dl.direction);
    lightUbo.color[i].head<3>() = dl.color;
    lightUbo.intensity[i].x() = dl.intensity;
  }

  ShadowAtlasInfoUbo shadowAtlasUbo{
      .regions{},
  };
  Mat4f lightViewProj[kNumLights];
  for (size_t i = 0; i < numLights; i++) {
    lightViewProj[i].noalias() = shadowPass.cam[i].viewProj();
    const auto &reg = shadowPass.regions[i];
    shadowAtlasUbo.regions[i] = {reg.x, reg.y, reg.w, reg.h};
  }
  const Mat4f viewProj = camera.viewProj();

  // if geometry is opaque, this is the first render pass, hence we clear the
  // color target transparent objects do not participate in SSAO
  SDL_GPURenderPass *render_pass;
  if (transparent) {
    render_pass = getTransparentRenderPass(m_renderer, command_buffer, gBuffer);
  } else {
    render_pass = getOpaqueRenderPass(
        m_renderer, command_buffer, SDL_GPU_LOADOP_CLEAR,
        pbrHasPrepass() ? SDL_GPU_LOADOP_LOAD : SDL_GPU_LOADOP_CLEAR, true,
        gBuffer);
  }

  command_buffer.pushFragmentUniform(FragmentUniformSlots::LIGHTING, lightUbo);
  if (!transparent) {
    if (shadowsEnabled()) {
      rend::bindFragmentSamplers(render_pass, SHADOW_MAP_SLOT,
                                 {{
                                     .texture = shadowPass.shadowMap,
                                     .sampler = shadowPass.sampler,
                                 }});
    }
    rend::bindFragmentSamplers(render_pass, SSAO_SLOT,
                               {{
                                   .texture = ssaoPass.ssaoMap,
                                   .sampler = ssaoPass.texSampler,
                               }});
    command_buffer.pushFragmentUniform(FragmentUniformSlots::ATLAS_INFO,
                                       shadowAtlasUbo);
  }

  auto view =
      m_registry.view<const TransformComponent, const MeshMaterialComponent,
                      pipeline_tag<PIPELINE_TRIANGLEMESH>>(
          entt::exclude<Disable>);

  // Transparent shader has baseColorTex as its only sampler (slot 0),
  // opaque shader has shadow(0), ssao(1), baseColor(2).
  const Uint32 baseColorSlot = transparent ? 0u : BASE_COLOR_TEX_SLOT;

  auto process_entities = [&](entt::entity ent) {
    auto [tr, obj] =
        m_registry.get<const TransformComponent, const MeshMaterialComponent>(
            ent);
    const Mat4f modelView = camera.view * tr;
    const Mesh &mesh = obj.mesh;
    const Mat4f mvp = viewProj * tr;
    TransformUniformData data{
        .modelView = modelView,
        .mvp = mvp,
        .normalMatrix = math::computeNormalMatrix(modelView),
    };
    command_buffer.pushVertexUniform(VertexUniformSlots::TRANSFORM, data);
    if (shadowsEnabled()) {
      LightSpaceMatricesUbo shadowUbo;
      shadowUbo.numLights = numLights;
      for (size_t i = 0; i < numLights; i++) {
        GpuMat4 lightMvp = lightViewProj[i] * tr;
        shadowUbo.mvps[i] = lightMvp;
      }
      command_buffer.pushVertexUniform(VertexUniformSlots::LIGHT_MATRICES,
                                       shadowUbo);
    }
    rend::bindMesh(render_pass, mesh);
    for (size_t j = 0; j < mesh.numViews(); j++) {
      command_buffer.pushFragmentUniform(FragmentUniformSlots::MATERIAL,
                                         obj.materials[j]);
      // Bind base color texture for this sub-mesh
      SDL_GPUTexture *tex = (j < obj.baseColorTextures.size())
                                ? (SDL_GPUTexture *)obj.baseColorTextures[j]
                                : (SDL_GPUTexture *)m_whiteTexture;
      rend::bindFragmentSamplers(render_pass, baseColorSlot,
                                 {{
                                     .texture = tex,
                                     .sampler = m_materialSampler,
                                 }});
      rend::drawView(render_pass, mesh.view(j));
    }
  };

  if (transparent) {
    auto *pipeline =
        m_pipelines.get({PIPELINE_TRIANGLEMESH, true, RenderMode::FILL});
    if (!pipeline) {
      SDL_EndGPURenderPass(render_pass);
      return;
    }
    pipeline->bind(render_pass);
    for (entt::entity ent :
         view | m_registry.view<entt::entity>(entt::exclude<Opaque>)) {
      process_entities(ent);
    }
  } else {
    auto opaques = view | m_registry.view<Opaque>();
    auto get_filter = [&reg = this->m_registry](RenderMode ref_mode) {
      return std::views::filter([&reg, ref_mode](entt::entity ent) {
        return reg.get<const MeshMaterialComponent>(ent).mode == ref_mode;
      });
    };

    if (auto pipeline =
            m_pipelines.get({PIPELINE_TRIANGLEMESH, false, RenderMode::FILL})) {
      pipeline->bind(render_pass);
      for (auto ent : opaques | get_filter(RenderMode::FILL)) {
        process_entities(ent);
      }
    }

    if (auto pipeline =
            m_pipelines.get({PIPELINE_TRIANGLEMESH, false, RenderMode::LINE})) {
      pipeline->bind(render_pass);
      for (auto ent : opaques | get_filter(RenderMode::LINE)) {
        process_entities(ent);
      }
    }
  }

  SDL_EndGPURenderPass(render_pass);
}

void RobotScene::renderOtherGeometry(CommandBuffer &command_buffer,
                                     const Camera &camera) {
  SDL_GPURenderPass *render_pass =
      getOpaqueRenderPass(m_renderer, command_buffer, SDL_GPU_LOADOP_LOAD,
                          SDL_GPU_LOADOP_LOAD, false, gBuffer);

  const Mat4f viewProj = camera.viewProj();

  // iterate over primitive types in the keys
  magic_enum::enum_for_each<PipelineType>([&](auto current_pipeline_type) {
    if (current_pipeline_type == PIPELINE_TRIANGLEMESH)
      return;

    auto *pipeline =
        m_pipelines.get({current_pipeline_type, false, RenderMode::FILL});
    if (!pipeline)
      return;
    pipeline->bind(render_pass);

    auto env_view =
        m_registry.view<const TransformComponent, const MeshMaterialComponent,
                        pipeline_tag<current_pipeline_type>>(
            entt::exclude<Disable>);
    for (auto &&[entity, tr, obj] : env_view.each()) {
      const Mesh &mesh = obj.mesh;
      const GpuMat4 mvp = viewProj * tr;
      const GpuVec4 &color = obj.materials[0].baseColor;
      command_buffer.pushVertexUniform(VertexUniformSlots::TRANSFORM, mvp)
          .pushFragmentUniform(FragmentUniformSlots::MATERIAL, color);
      rend::bindMesh(render_pass, mesh);
      rend::draw(render_pass, mesh);
    }
  });
  SDL_EndGPURenderPass(render_pass);
}

void RobotScene::release() {
  if (!device())
    return;

  clearEnvironment();
  clearRobotGeometries();

  m_pipelines.clear();
  m_wboitComposite.release();

  gBuffer.release();
  ssaoPass.release();
  shadowPass.release();

  m_whiteTexture.destroy();
  if (m_materialSampler) {
    SDL_ReleaseGPUSampler(device(), m_materialSampler);
    m_materialSampler = nullptr;
  }
}

static RobotScene::PipelineConfig
getPipelineConfig(const RobotScene::Config &cfg, RobotScene::PipelineType type,
                  bool transparent) {
  using enum RobotScene::PipelineType;
  switch (type) {
  case PIPELINE_TRIANGLEMESH:
    return transparent ? cfg.triangle_config.transparent
                       : cfg.triangle_config.opaque;
  case PIPELINE_HEIGHTFIELD:
    return cfg.heightfield_config;
  case PIPELINE_POINTCLOUD:
    return cfg.pointcloud_config;
  }
}

GraphicsPipeline
RobotScene::createRenderPipeline(const PipelineKey &key,
                                 const MeshLayout &layout,
                                 SDL_GPUTextureFormat render_target_format,
                                 SDL_GPUTextureFormat depth_stencil_format) {
  assert(validateMeshLayout(layout));

  auto [type, transparent, renderMode] = key;
  const auto sample_count = m_renderer.getMsaaSampleCount();
  spdlog::info("Building pipeline for type {:s} ({:d} MSAA)",
               magic_enum::enum_name(type), sdlSampleToValue(sample_count));

  PipelineConfig pipe_config = getPipelineConfig(m_config, type, transparent);
  auto vertexShader =
      Shader::fromMetadata(device(), pipe_config.vertex_shader_path);
  auto fragmentShader =
      Shader::fromMetadata(device(), pipe_config.fragment_shader_path);

  SDL_GPUColorTargetDescription color_targets[3];
  SDL_zero(color_targets);
  color_targets[0].format = render_target_format;
  color_targets[0].blend_state = {
      .src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA,
      .dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
      .color_blend_op = SDL_GPU_BLENDOP_ADD,
      .src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA,
      .dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
      .alpha_blend_op = SDL_GPU_BLENDOP_ADD,
      .enable_blend = true,
  };
  color_targets[1].format = gBuffer.normalMap.format();
  bool had_prepass = (type == PIPELINE_TRIANGLEMESH) && pbrHasPrepass();
  SDL_GPUCompareOp depth_compare_op = SDL_GPU_COMPAREOP_LESS_OR_EQUAL;

  SDL_GPUFillMode fill_mode;
  switch (renderMode) {
    using enum RenderMode;
  case FILL:
    fill_mode = SDL_GPU_FILLMODE_FILL;
    break;
  case LINE:
    fill_mode = SDL_GPU_FILLMODE_LINE;
    break;
  }

  SDL_GPUGraphicsPipelineCreateInfo desc{
      .vertex_shader = vertexShader,
      .fragment_shader = fragmentShader,
      .vertex_input_state = layout,
      .primitive_type = getPrimitiveTopologyForType(type),
      .rasterizer_state{
          .fill_mode = fill_mode,
          .cull_mode = pipe_config.cull_mode,
      },
      .depth_stencil_state{
          .compare_op = depth_compare_op,
          .enable_depth_test = true,
          // no depth write if there was a prepass
          .enable_depth_write = !had_prepass,
      },
      .target_info{
          .color_target_descriptions = color_targets,
          .num_color_targets = 2,
          .depth_stencil_format = depth_stencil_format,
          .has_depth_stencil_target = true,
      },
  };
  desc.multisample_state.sample_count = sample_count;

  if (type == PIPELINE_TRIANGLEMESH) {
    if (transparent) {
      // modify color targets
      SDL_zero(color_targets);
      // Accumulation target with additive blending
      color_targets[0].format = gBuffer.accumTexture.format();
      color_targets[0].blend_state = {
          .src_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE,
          .dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE,
          .color_blend_op = SDL_GPU_BLENDOP_ADD,
          .src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE,
          .dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE,
          .alpha_blend_op = SDL_GPU_BLENDOP_ADD,
          .enable_blend = true,
      };

      // Revealage target with multiplicative blending
      color_targets[1].format = gBuffer.revealTexture.format();
      color_targets[1].blend_state = {
          .src_color_blendfactor = SDL_GPU_BLENDFACTOR_ZERO,
          .dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_COLOR,
          .color_blend_op = SDL_GPU_BLENDOP_ADD,
          .src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO,
          .dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
          .alpha_blend_op = SDL_GPU_BLENDOP_ADD,
          .enable_blend = true,
      };

      desc.depth_stencil_state.enable_depth_write = false;
    } else {
      // Opaque triangle mesh: add depth copy as 3rd G-buffer color target
      color_targets[2].format = gBuffer.depthCopyTex.format();
      desc.target_info.num_color_targets = 3;
    }

    spdlog::info(" > transparency:  {}", transparent);
    spdlog::info(" > render mode:   {:s}", magic_enum::enum_name(renderMode));
    spdlog::info(" > depth comp op: {:s}",
                 magic_enum::enum_name(depth_compare_op));
    spdlog::info(" > prepass:       {}", had_prepass);
  }

  return GraphicsPipeline(device(), desc, nullptr);
}

} // namespace candlewick::multibody
