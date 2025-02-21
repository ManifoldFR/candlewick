#pragma once

#include "Multibody.h"
#include "../core/Device.h"
#include "../core/Scene.h"
#include "../core/LightUniforms.h"
#include "../core/AABB.h"
#include "../core/DepthAndShadowPass.h"
#include "../core/Texture.h"
#include "../core/Components.h"
#include "../posteffects/SSAO.h"
#include "../utils/MeshData.h"
#include "../third-party/magic_enum.hpp"

#include <entt/entity/fwd.hpp>
#include <coal/fwd.hh>
#include <pinocchio/multibody/fwd.hpp>

namespace candlewick {
namespace multibody {

  void updateRobotTransforms(entt::registry &registry,
                             const pin::GeometryData &geom_data);

  /// \brief A scene/render system for robot geometries using Pinocchio.
  class RobotScene {
  public:
    enum PipelineType {
      PIPELINE_TRIANGLEMESH,
      PIPELINE_HEIGHTFIELD,
      PIPELINE_POINTCLOUD,
    };
    static constexpr size_t kNumPipelineTypes =
        magic_enum::enum_count<PipelineType>();
    enum VertexUniformSlots : Uint32 { TRANSFORM = 0 };
    enum FragmentUniformSlots : Uint32 { MATERIAL = 0, LIGHTING = 1 };

    /// Map hpp-fcl/coal collision geometry to desired pipeline type.
    static PipelineType pinGeomToPipeline(const coal::CollisionGeometry &geom);

    /// Map pipeline type to geometry primitive.
    static constexpr SDL_GPUPrimitiveType
    getPrimitiveTopologyForType(PipelineType type) {
      switch (type) {
      case PIPELINE_TRIANGLEMESH:
        return SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
      case PIPELINE_HEIGHTFIELD:
        return SDL_GPU_PRIMITIVETYPE_LINELIST;
      case PIPELINE_POINTCLOUD:
        return SDL_GPU_PRIMITIVETYPE_POINTLIST;
      }
    }

    struct MeshMaterialComponent {
      Mesh mesh;
      std::vector<PbrMaterial> materials;
      PipelineType pipeline_type;
      MeshMaterialComponent(Mesh &&mesh, std::vector<PbrMaterial> &&materials,
                            PipelineType pipelineType)
          : mesh(std::move(mesh)), materials(std::move(materials)),
            pipeline_type(pipelineType) {
        assert(mesh.numViews() == materials.size());
      }
    };

    struct PipelineConfig {
      // shader set
      const char *vertex_shader_path;
      const char *fragment_shader_path;
      SDL_GPUCullMode cull_mode = SDL_GPU_CULLMODE_BACK;
      SDL_GPUFillMode fill_mode = SDL_GPU_FILLMODE_FILL;
    };
    struct Config {
      std::unordered_map<PipelineType, PipelineConfig> pipeline_configs = {
          {PIPELINE_TRIANGLEMESH,
           {
               .vertex_shader_path = "PbrBasic.vert",
               .fragment_shader_path = "PbrBasic.frag",
           }},
          {PIPELINE_HEIGHTFIELD,
           {
               .vertex_shader_path = "Hud3dElement.vert",
               .fragment_shader_path = "Hud3dElement.frag",
           }},
          // {PIPELINE_POINTCLOUD, {}}
      };
      bool enable_msaa = false;
      bool enable_shadows = true;
      bool enable_ssao = true;
      bool triangle_has_prepass = false;
      bool enable_normal_target = false;
      SDL_GPUSampleCount msaa_samples = SDL_GPU_SAMPLECOUNT_1;
      ShadowPassConfig shadow_config;
    };

    RobotScene(entt::registry &registry, const Renderer &renderer,
               const pin::GeometryModel &geom_model,
               const pin::GeometryData &geom_data, Config config);

    void updateTransforms();

    void collectOpaqueCastables();
    const std::vector<OpaqueCastable> &castables() const { return _castables; }

    entt::entity
    addEnvironmentObject(MeshData &&data, Mat4f placement,
                         PipelineType pipe_type = PIPELINE_TRIANGLEMESH);

    entt::entity
    addEnvironmentObject(MeshData &&data, const Eigen::Affine3f &T,
                         PipelineType pipe_type = PIPELINE_TRIANGLEMESH) {
      return addEnvironmentObject(std::move(data), T.matrix(), pipe_type);
    }

    void clearEnvironment();
    void clearRobotGeometries();

    [[nodiscard]] SDL_GPUGraphicsPipeline *createPipeline(
        const MeshLayout &layout, SDL_GPUTextureFormat render_target_format,
        SDL_GPUTextureFormat depth_stencil_format, PipelineType type);

    /// \warning Call updateRobotTransforms() before rendering the objects with
    /// this function.
    void render(CommandBuffer &command_buffer, const Camera &camera);
    /// \brief PBR render pass for triangle meshes.
    void renderPBRTriangleGeometry(CommandBuffer &command_buffer,
                                   const Camera &camera);
    /// \brief Render pass for other geometry.
    void renderOtherGeometry(CommandBuffer &command_buffer,
                             const Camera &camera);
    void release();

    Config &config() { return _config; }
    const Config &config() const { return _config; }
    inline bool pbrHasPrepass() const { return _config.triangle_has_prepass; }
    inline bool shadowsEnabled() const { return _config.enable_shadows; }

    /// \brief Getter for the referenced pinocchio GeometryModel object.
    const pin::GeometryModel &geomModel() const { return _geomModel; }

    /// \brief Getter for the referenced pinocchio GeometryData object.
    const pin::GeometryData &geomData() const { return _geomData; }

    const entt::registry &registry() const { return _registry; }

    const Device &device() { return _renderer.device; }

    void initGBuffer(const Renderer &renderer);

    SDL_GPUGraphicsPipeline *renderPipelines[kNumPipelineTypes];
    DirectionalLight directionalLight;
    ssao::SsaoPass ssaoPass{NoInit};
    struct GBuffer {
      Texture normalMap{NoInit};
    } gBuffer;
    ShadowPassInfo shadowPass;
    AABB worldSpaceBounds;

  private:
    entt::registry &_registry;
    Config _config;
    const Renderer &_renderer;
    std::reference_wrapper<pin::GeometryModel const> _geomModel;
    std::reference_wrapper<pin::GeometryData const> _geomData;
    std::vector<OpaqueCastable> _castables;
  };
  static_assert(Scene<RobotScene>);

} // namespace multibody
} // namespace candlewick
