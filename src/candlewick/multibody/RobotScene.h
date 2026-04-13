/// \file Visualizer.h
/// \author ManifoldFR
/// \copyright 2025 INRIA
#pragma once

#include "Multibody.h"

#include "../core/Device.h"
#include "../core/RenderContext.h"
#include "../core/LightUniforms.h"
#include "../core/DepthAndShadowPass.h"
#include "../core/Texture.h"
#include "../posteffects/SSAO.h"
#include "../utils/MeshData.h"

#include <magic_enum/magic_enum.hpp>
#include <entt/entity/fwd.hpp>
#include <coal/fwd.hh>
#include <pinocchio/multibody/fwd.hpp>
#include <set>

namespace candlewick {
enum class RenderMode;

namespace multibody {

  /// \brief A system for updating the transform components for robot geometry
  /// entities.
  ///
  /// This will also update the mesh materials.
  ///
  /// Reads PinGeomObjComponent, updates TransformComponent.
  void updateRobotTransforms(entt::registry &registry,
                             const pin::GeometryModel &geom_model,
                             const pin::GeometryData &geom_data);

  /// \brief A render system for Pinocchio robot geometries using Pinocchio.
  ///
  /// This internally stores references to pinocchio::GeometryModel and
  /// pinocchio::GeometryData objects.
  class RobotScene final {
    [[nodiscard]] bool hasInternalPointers() const {
      return m_geomModel && m_geomData;
    }

    void compositeTransparencyPass(CommandBuffer &command_buffer);

    void renderPBRTriangleGeometry(CommandBuffer &command_buffer,
                                   const Camera &camera, bool transparent);

    void renderOtherGeometry(CommandBuffer &command_buffer,
                             const Camera &camera);

    void initGBuffer();

    void initCompositePipeline(const MeshLayout &layout);

  public:
    enum class PipelineType {
      PIPELINE_TRIANGLEMESH,
      PIPELINE_HEIGHTFIELD,
      PIPELINE_POINTCLOUD,
    };
    using enum PipelineType;
    enum VertexUniformSlots : Uint32 { TRANSFORM, LIGHT_MATRICES };
    enum FragmentUniformSlots : Uint32 {
      MATERIAL,
      LIGHTING,
      ATLAS_INFO,
    };
    enum FragmentSamplerSlots : Uint32 {
      SHADOW_MAP_SLOT,
      SSAO_SLOT,
      BASE_COLOR_TEX_SLOT
    };

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

    template <PipelineType T> using pipeline_tag = entt::integral_constant<T>;

    struct PipelineConfig {
      // shader set
      const char *vertex_shader_path;
      const char *fragment_shader_path;
      SDL_GPUCullMode cull_mode = SDL_GPU_CULLMODE_BACK;
    };
    struct Config {
      struct TrianglePipelineConfig {
        PipelineConfig opaque{
            .vertex_shader_path = "PbrBasic.vert",
            .fragment_shader_path = "PbrBasic.frag",
        };
        PipelineConfig transparent{
            .vertex_shader_path = "PbrBasic.vert",
            .fragment_shader_path = "PbrTransparent.frag",
            .cull_mode = SDL_GPU_CULLMODE_NONE,
        };
      } triangle_config;
      PipelineConfig heightfield_config{
          .vertex_shader_path = "Hud3dElement.vert",
          .fragment_shader_path = "Hud3dElement.frag",
      };
      PipelineConfig pointcloud_config;
      bool enable_shadows = true;
      bool enable_ssao = true;
      bool triangle_has_prepass = false;
      Uint32 ssao_kernel_size = 16u;
      ShadowPassConfig shadow_config;
    };

    struct PipelineKey {
      PipelineType type;
      bool transparent;
      RenderMode renderMode;

      auto operator<=>(const PipelineKey &) const = default;
    };
    class PipelineManager {
      std::map<PipelineKey, GraphicsPipeline> m_store;

    public:
      PipelineManager() : m_store() {}

      bool contains(const PipelineKey &key) const {
        return m_store.contains(key);
      }

      PipelineManager(const PipelineManager &) = delete;
      PipelineManager(PipelineManager &&) = default;
      PipelineManager &operator=(const PipelineManager &) = delete;

      GraphicsPipeline *get(const PipelineKey &key) {
        auto it = m_store.find(key);
        if (it != m_store.cend())
          return &it->second;

        return nullptr;
      }

      void set(const PipelineKey &key, GraphicsPipeline &&pipeline) {
        m_store.emplace(key, std::move(pipeline));
      }

      void clear() { m_store.clear(); }
    };

    std::array<DirectionalLight, kNumLights> directionalLight;
    ssao::SsaoPass ssaoPass{NoInit};
    struct GBuffer {
      Texture normalMap{NoInit};
      Texture resolveNormalMap{NoInit};
      // Depth copy as color target (avoids sampling MSAA depth stencil texture)
      Texture depthCopyTex{NoInit};
      Texture resolveDepthCopyTex{NoInit};

      // WBOIT buffers
      Texture accumTexture{NoInit};
      Texture revealTexture{NoInit};
      Texture resolveAccumTexture{NoInit};
      Texture resolveRevealTexture{NoInit};
      SDL_GPUSampler *sampler = nullptr; // composite pass

      bool initialized() const {
        return (sampler != nullptr) && normalMap && accumTexture &&
               revealTexture;
      }

      void release() noexcept {
        auto *device = normalMap.device();
        normalMap.destroy();
        resolveNormalMap.destroy();
        depthCopyTex.destroy();
        resolveDepthCopyTex.destroy();
        accumTexture.destroy();
        revealTexture.destroy();
        resolveAccumTexture.destroy();
        resolveRevealTexture.destroy();
        if (sampler) {
          SDL_ReleaseGPUSampler(device, sampler);
          sampler = nullptr;
        }
      }
      ~GBuffer() noexcept { this->release(); }
    } gBuffer;
    ShadowMapPass shadowPass{NoInit};

    /// \brief Non-initializing constructor.
    RobotScene(entt::registry &registry, const RenderContext &renderer);

    /// \brief Constructor which initializes the system.
    ///
    /// loadModels() will be called.
    RobotScene(entt::registry &registry, const RenderContext &renderer,
               const pin::GeometryModel &geom_model,
               const pin::GeometryData &geom_data, Config config);

    RobotScene(const RobotScene &) = delete;

    void setConfig(const Config &config) {
      if (m_initialized)
        terminate_with_message(
            "Cannot call setConfig() after render system was initialized.");

      m_config = config;
    }

    /// \brief Set the internal geometry model and data pointers, and load the
    /// corresponding models.
    void loadModels(const pin::GeometryModel &geom_model,
                    const pin::GeometryData &geom_data);

    /// \brief Update the transform component of the GeometryObject entities.
    void update();

    void collectOpaqueCastables();
    const std::vector<OpaqueCastable> &castables() const { return m_castables; }

    Uint32 numLights() const noexcept { return shadowPass.numLights(); }

    entt::entity
    addEnvironmentObject(MeshData &&data, Mat4f placement,
                         PipelineType pipe_type = PIPELINE_TRIANGLEMESH);

    entt::entity
    addEnvironmentObject(MeshData &&data, const Eigen::Affine3f &T,
                         PipelineType pipe_type = PIPELINE_TRIANGLEMESH) {
      return addEnvironmentObject(std::move(data), T.matrix(), pipe_type);
    }

    /// \brief Destroy all entities with the EnvironmentTag component.
    void clearEnvironment();
    /// \brief Destroy all entities with the PinGeomObjComponent component
    /// (Pinocchio geometry objects).
    void clearRobotGeometries();

    GraphicsPipeline
    createRenderPipeline(const PipelineKey &key, const MeshLayout &layout,
                         SDL_GPUTextureFormat render_target_format,
                         SDL_GPUTextureFormat depth_stencil_format);

    /// \warning Call updateTransforms() before rendering the objects with
    /// this function.
    void render(CommandBuffer &command_buffer, const Camera &camera);

    void renderOpaque(CommandBuffer &command_buffer, const Camera &camera);

    void renderTransparent(CommandBuffer &command_buffer, const Camera &camera);

    /// \brief Release all resources.
    void release();

    Config &config() { return m_config; }
    const Config &config() const { return m_config; }
    inline bool pbrHasPrepass() const { return m_config.triangle_has_prepass; }
    inline bool shadowsEnabled() const { return m_config.enable_shadows; }

    using pipeline_req_t = std::tuple<MeshLayout, PipelineKey>;
    /// \brief Ensure the render pipelines were properly created following the
    /// provided requirements.
    void
    ensurePipelinesExist(const std::set<pipeline_req_t> &required_pipelines);

    /// \brief Getter for the pinocchio GeometryModel object.
    const pin::GeometryModel &geomModel() const { return *m_geomModel; }

    /// \brief Getter for the pinocchio GeometryData object.
    const pin::GeometryData &geomData() const { return *m_geomData; }

    const Device &device() { return m_renderer.device; }
    entt::registry &registry() { return m_registry; }
    const entt::registry &registry() const { return m_registry; }

  private:
    entt::registry &m_registry;
    const RenderContext &m_renderer;
    Config m_config;
    const pin::GeometryModel *m_geomModel;
    const pin::GeometryData *m_geomData;
    std::vector<OpaqueCastable> m_castables;
    bool m_initialized;
    PipelineManager m_pipelines;
    GraphicsPipeline m_wboitComposite{NoInit};
    Texture m_whiteTexture{NoInit};
    SDL_GPUSampler *m_materialSampler = nullptr;
  };
  static_assert(Scene<RobotScene>);

} // namespace multibody
} // namespace candlewick
