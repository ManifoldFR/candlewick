#include "Common.h"

#include "candlewick/core/Renderer.h"
#include "candlewick/core/GuiSystem.h"
#include "candlewick/core/Shader.h"
#include "candlewick/core/CameraControls.h"
#include "candlewick/core/LightUniforms.h"
#include "candlewick/core/MaterialUniform.h"
#include "candlewick/core/TransformUniforms.h"

#include "candlewick/utils/WriteTextureToImage.h"
#include "candlewick/utils/MeshData.h"
#include "candlewick/multibody/LoadPinocchioGeometry.h"

#include "candlewick/primitives/Arrow.h"
#include "candlewick/primitives/Plane.h"
#include "candlewick/primitives/Grid.h"

#include <imgui.h>
#include <imgui_impl_sdl3.h>

#include <robot_descriptions_cpp/robot_load.hpp>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>

#include <SDL3/SDL_log.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_events.h>
#include <SDL3/SDL_assert.h>
#include <SDL3/SDL_gpu.h>

#include <CLI/App.hpp>
#include <CLI/Formatter.hpp>
#include <CLI/Config.hpp>

namespace pin = pinocchio;
using namespace candlewick;

/// Application constants

constexpr Uint32 wWidth = 1600;
constexpr Uint32 wHeight = 900;
constexpr float aspectRatio = float(wWidth) / float(wHeight);

/// Application state

static bool renderPlane = true;
static bool renderGrid = true;
static Camera g_camera;
static CylinderCameraControl g_camControl{g_camera};
static CameraProjection g_cameraType = CameraProjection::PERSPECTIVE;
// Current perspective matrix fov
static Radf currentFov = 55.0_degf;
// Current ortho matrix scale
static float currentOrthoScale = 1.f;
static bool quitRequested = false;

static float pixelDensity;
static float displayScale;

static GpuVec4 gridColor = 0xE0A236ff_rgbaf;
static Mesh gridMesh{NoInit};

static DirectionalLight g_myLight = {
    .direction = {0., -1., -1.},
    .color = {1.0, 1.0, 1.0},
    .intensity = 8.0,
};

struct alignas(16) light_ubo_t {
  GpuVec3 viewSpaceDir;
  alignas(16) GpuVec3 color;
  float intensity;
};

static void updateFov(Radf newFov) {
  g_camera.projection = perspectiveFromFov(newFov, aspectRatio, 0.01f, 10.0f);
  currentFov = newFov;
}

static void updateOrtho(float zoom) {
  float iz = 1.f / zoom;
  g_camera.projection = orthographicMatrix({iz * aspectRatio, iz}, 0.01f, 4.f);
  currentOrthoScale = zoom;
  auto pos = g_camera.position();
  SDL_Log("Current ortho cam. pos: (%f, %f, %f)", pos.x(), pos.y(), pos.z());
}

void eventLoop(const Renderer &renderer) {
  // update pixel density and display scale
  pixelDensity = renderer.window.pixelDensity();
  displayScale = renderer.window.displayScale();
  const float rotSensitivity = 5e-3f * pixelDensity;
  const float panSensitivity = 1e-2f * pixelDensity;
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL3_ProcessEvent(&event);
    const bool controlPressed = SDL_GetModState() & SDL_KMOD_CTRL;
    if (event.type == SDL_EVENT_QUIT) {
      SDL_Log("Application exit requested.");
      quitRequested = true;
      break;
    }

    ImGuiIO &io = ImGui::GetIO();
    if (io.WantCaptureMouse | io.WantCaptureKeyboard)
      continue;
    switch (event.type) {
    case SDL_EVENT_MOUSE_WHEEL: {
      const float scaleFac = std::exp(kScrollZoom * event.wheel.y);
      switch (g_cameraType) {
      case CameraProjection::ORTHOGRAPHIC:
        updateOrtho(std::clamp(scaleFac * currentOrthoScale, 0.1f, 2.f));
        break;
      case CameraProjection::PERSPECTIVE:
        updateFov(Radf(std::min(currentFov * scaleFac, Radf{170.0_degf})));
        break;
      }
      break;
    }
    case SDL_EVENT_KEY_DOWN: {
      const float step_size = 0.06f;
      switch (event.key.key) {
      case SDLK_LEFT:
        camera_util::localTranslate(g_camera, {+step_size, 0, 0});
        break;
      case SDLK_RIGHT:
        camera_util::localTranslate(g_camera, {-step_size, 0, 0});
        break;
      case SDLK_UP:
        g_camControl.dolly(+step_size);
        break;
      case SDLK_DOWN:
        g_camControl.dolly(-step_size);
        break;
      }
      break;
    }
    case SDL_EVENT_MOUSE_MOTION: {
      SDL_MouseButtonFlags mouseButton = event.motion.state;
      if (mouseButton & SDL_BUTTON_LMASK) {
        if (controlPressed) {
          g_camControl.moveInOut(0.95f, event.motion.yrel);
        } else {
          g_camControl.viewportDrag(
              Float2{event.motion.xrel, event.motion.yrel}, rotSensitivity,
              panSensitivity);
        }
      }
      if (mouseButton & SDL_BUTTON_RMASK) {
        float camXLocRotSpeed = 0.01f * pixelDensity;
        camera_util::localRotateXAroundOrigin(g_camera, camXLocRotSpeed *
                                                            event.motion.yrel);
      }
      break;
    }
    }
  }
}

Renderer createRenderer(Uint32 width, Uint32 height,
                        SDL_GPUTextureFormat depth_stencil_format) {
  Device device{auto_detect_shader_format_subset(), true};
  return Renderer{std::move(device),
                  Window{__FILE__, int(width), int(height), 0},
                  depth_stencil_format};
}

int main(int argc, char **argv) {
  SDL_GPUGraphicsPipeline *meshPipeline;

  CLI::App app{"Ur5 example"};
  bool performRecording{false};
  argv = app.ensure_utf8(argv);
  app.add_flag("-r,--record", performRecording, "Record output");
  CLI11_PARSE(app, argc, argv);

  if (!SDL_Init(SDL_INIT_VIDEO))
    return 1;

  Renderer renderer =
      createRenderer(wWidth, wHeight, SDL_GPU_TEXTUREFORMAT_D24_UNORM);
  Device &device = renderer.device;

  // Load plane
  MeshData plane_data = loadPlaneTiled(0.25f, 5, 5);
  Mesh plane = createMesh(device, plane_data);
  uploadMeshToDevice(device, plane, plane_data);

  // Load grid
  MeshData grid_data = loadGrid(10);
  gridMesh = createMesh(device, grid_data);
  uploadMeshToDevice(device, gridMesh, grid_data);

  std::array triad_data = loadTriadSolid();
  std::vector<Mesh> triad_meshes;
  for (auto &&arrow_data : std::move(triad_data)) {
    Mesh arrow_mesh = createMesh(device, arrow_data);
    uploadMeshToDevice(device, arrow_mesh, arrow_data);
    triad_meshes.push_back(std::move(arrow_mesh));
  }

  GuiSystem guiSys{
      renderer, [&plane_data](const Renderer &r) {
        static bool demo_window_open = true;

        ImGui::Begin("Renderer info & controls", nullptr,
                     ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Device driver: %s", r.device.driverName());
        ImGui::SeparatorText("Camera");
        ImGui::RadioButton("Orthographic", (int *)&g_cameraType,
                           int(CameraProjection::ORTHOGRAPHIC));
        ImGui::SameLine();
        ImGui::RadioButton("Perspective", (int *)&g_cameraType,
                           int(CameraProjection::PERSPECTIVE));
        switch (g_cameraType) {
        case CameraProjection::ORTHOGRAPHIC:
          if (ImGui::DragFloat("zoom", &currentOrthoScale, 0.01f, 0.1f, 2.f,
                               "%.3f", ImGuiSliderFlags_AlwaysClamp))
            updateOrtho(currentOrthoScale);
          break;
        case CameraProjection::PERSPECTIVE:
          Degf newFov{currentFov};
          if (ImGui::DragFloat("fov", newFov, 1.f, 15.f, 90.f, "%.3f",
                               ImGuiSliderFlags_AlwaysClamp))
            updateFov(Radf(newFov));
          break;
        }

        ImGui::SeparatorText("Env. status");
        ImGui::Checkbox("Render plane", &renderPlane);
        ImGui::Checkbox("Render grid", &renderGrid);

        ImGui::SeparatorText("Lights");
        ImGui::DragFloat("intens.", &g_myLight.intensity, 0.1f, 0.1f, 10.0f);
        ImGui::ColorEdit3("color", g_myLight.color.data());
        ImGui::Separator();
        ImGui::ColorEdit4("grid color", gridColor.data(),
                          ImGuiColorEditFlags_AlphaPreview);
        ImGui::ColorEdit4("plane color", plane_data.material.baseColor.data());
        ImGui::End();
        ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once);
        ImGui::ShowDemoWindow(&demo_window_open);
      }};

  // Load robot
  pin::Model model;
  pin::GeometryModel geom_model;
  robot_descriptions::loadModelsFromToml("ur.toml", "ur5_gripper", model,
                                         &geom_model, NULL);
  pin::Data pin_data{model};
  pin::GeometryData geom_data{geom_model};

  struct RobotObject {
    pin::GeomIndex geom_index;
    Mesh mesh;
    std::vector<PbrMaterial> materials;
  };
  std::vector<RobotObject> robotShapes;
  for (size_t i = 0; i < geom_model.ngeoms; i++) {
    const auto &gobj = geom_model.geometryObjects[i];
    /// Create a Mesh, upload to device
    auto meshDatas = multibody::loadGeometryObject(gobj);
    auto mesh = createMeshFromBatch(device, meshDatas, true);

    robotShapes.push_back({i, std::move(mesh), extractMaterials(meshDatas)});
    SDL_Log("Loaded %zu MeshData objects. Meshes:", meshDatas.size());
    for (size_t i = 0; i < meshDatas.size(); i++) {
      SDL_Log("   [%zu] %u vertices, %u indices", i, meshDatas[i].numVertices(),
              meshDatas[i].numIndices());
    }
  }
  SDL_Log("Created %zu robot mesh shapes.", robotShapes.size());

  /** CREATE PIPELINES **/
  // Robot mesh pipeline
  const SDL_GPUTextureFormat swapchain_format =
      renderer.getSwapchainTextureFormat();
  {
    Shader vertexShader = Shader::fromMetadata(device, "PbrBasic.vert");
    Shader fragmentShader = Shader::fromMetadata(device, "PbrBasic.frag");

    SDL_GPUColorTargetDescription colorTarget;
    colorTarget.format = swapchain_format;
    SDL_zero(colorTarget.blend_state);

    // create pipeline
    SDL_GPUGraphicsPipelineCreateInfo mesh_pipeline_desc{
        .vertex_shader = vertexShader,
        .fragment_shader = fragmentShader,
        .vertex_input_state = robotShapes[0].mesh.layout.toVertexInputState(),
        .primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
        .depth_stencil_state{.compare_op = SDL_GPU_COMPAREOP_LESS_OR_EQUAL,
                             .enable_depth_test = true,
                             .enable_depth_write = true},
        .target_info{.color_target_descriptions = &colorTarget,
                     .num_color_targets = 1,
                     .depth_stencil_format = renderer.depthFormat(),
                     .has_depth_stencil_target = true},
        .props = 0,
    };

    meshPipeline = SDL_CreateGPUGraphicsPipeline(device, &mesh_pipeline_desc);
    if (meshPipeline == nullptr) {
      SDL_Log("Failed to create pipeline: %s", SDL_GetError());
      return 1;
    }

    vertexShader.release();
    fragmentShader.release();
  }

  SDL_GPUGraphicsPipeline *debugLinePipeline =
      initGridPipeline(renderer.device, renderer.window, gridMesh.layout,
                       renderer.depthFormat(), SDL_GPU_PRIMITIVETYPE_LINELIST);
  SDL_GPUGraphicsPipeline *debugTrianglePipeline = initGridPipeline(
      renderer.device, renderer.window, triad_meshes[0].layout,
      renderer.depthFormat(), SDL_GPU_PRIMITIVETYPE_TRIANGLELIST);

  // MAIN APPLICATION LOOP

  Uint32 frameNo = 0;

  g_camera.view = lookAt({2.0, 0, 2.}, Float3::Zero());
  updateFov(currentFov);

  Eigen::VectorXd q0 = pin::neutral(model);
  Eigen::VectorXd q1 = pin::randomConfiguration(model);

#ifdef CANDLEWICK_WITH_FFMPEG_SUPPORT
  media::VideoRecorder recorder{NoInit};
  if (performRecording)
    recorder = media::VideoRecorder{wWidth, wHeight, "ur5.mp4"};
#endif

  auto record_callback = [&] {
#ifdef CANDLEWICK_WITH_FFMPEG_SUPPORT
    auto swapchain_format = renderer.getSwapchainTextureFormat();
    media::videoWriteTextureToFrame(renderer.device, recorder,
                                    renderer.swapchain, swapchain_format,
                                    wWidth, wHeight);
#endif
  };

  while (frameNo < 5000 && !quitRequested) {
    // logic
    eventLoop(renderer);
    double phi = 0.5 * (1. + std::sin(frameNo * 1e-2));
    Eigen::VectorXd q = pin::interpolate(model, q0, q1, phi);
    pin::forwardKinematics(model, pin_data, q);
    pin::updateGeometryPlacements(model, pin_data, geom_model, geom_data);

    // acquire command buffer and swapchain
    auto command_buffer = renderer.acquireCommandBuffer();
    renderer.acquireSwapchain(command_buffer);
    SDL_GPURenderPass *render_pass;

    if (renderer.swapchain) {

      SDL_GPUColorTargetInfo ctinfo{
          .texture = renderer.swapchain,
          .clear_color = SDL_FColor{0., 0., 0., 0.},
          .load_op = SDL_GPU_LOADOP_CLEAR,
          .store_op = SDL_GPU_STOREOP_STORE,
          .cycle = false,
      };
      SDL_GPUDepthStencilTargetInfo depthTarget;
      SDL_zero(depthTarget);
      depthTarget.clear_depth = 1.0;
      depthTarget.load_op = SDL_GPU_LOADOP_CLEAR;
      depthTarget.store_op = SDL_GPU_STOREOP_DONT_CARE;
      depthTarget.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
      depthTarget.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
      depthTarget.texture = renderer.depth_texture;

      render_pass =
          SDL_BeginGPURenderPass(command_buffer, &ctinfo, 1, &depthTarget);

      SDL_assert(robotShapes.size() == geom_model.ngeoms);

      /// Model-view-projection (MVP) matrix
      Mat4f modelView;
      Mat4f mvp;

      const light_ubo_t lightUbo{
          g_camera.transformVector(g_myLight.direction),
          g_myLight.color,
          g_myLight.intensity,
      };
      SDL_BindGPUGraphicsPipeline(render_pass, meshPipeline);
      command_buffer.pushFragmentUniform(1, &lightUbo, sizeof(lightUbo));

      // loop over mesh groups
      for (size_t i = 0; i < geom_model.ngeoms; i++) {
        const Mat4f placement{geom_data.oMg[i].cast<float>()};
        modelView = g_camera.view * placement;
        mvp = g_camera.projection * modelView;
        TransformUniformData cameraUniform{
            modelView,
            mvp,
            math::computeNormalMatrix(modelView),
        };
        const auto &obj = robotShapes[i];
        const auto &mesh = obj.mesh;

        command_buffer.pushVertexUniform(0, &cameraUniform,
                                         sizeof(cameraUniform));
        rend::bindMesh(render_pass, mesh);
        for (size_t j = 0; j < mesh.numViews(); j++) {
          const auto material = obj.materials[j];
          command_buffer.pushFragmentUniform(0, &material, sizeof(material));
          rend::drawView(render_pass, mesh.view(j));
        }
      }

      // RENDER PLANE
      if (renderPlane) {
        Eigen::Affine3f plane_transform{Eigen::UniformScaling<float>(3.0f)};
        modelView = g_camera.view * plane_transform.matrix();
        mvp.noalias() = g_camera.projection * modelView;
        TransformUniformData cameraUniform{
            modelView,
            mvp,
            math::computeNormalMatrix(modelView),
        };
        const auto material = plane_data.material;
        command_buffer
            .pushVertexUniform(0, &cameraUniform, sizeof(cameraUniform))
            .pushFragmentUniform(0, &material, sizeof(material));
        rend::bindMesh(render_pass, plane);
        rend::draw(render_pass, plane);
      }

      // render 3d hud elements
      if (renderGrid) {
        const GpuMat4 cameraUniform = g_camera.viewProj();
        command_buffer
            .pushVertexUniform(0, &cameraUniform, sizeof(cameraUniform))
            .pushFragmentUniform(0, &gridColor, sizeof(gridColor));
        SDL_BindGPUGraphicsPipeline(render_pass, debugLinePipeline);
        rend::bindMesh(render_pass, gridMesh);
        rend::draw(render_pass, gridMesh);

        SDL_BindGPUGraphicsPipeline(render_pass, debugTrianglePipeline);
        for (size_t i = 0; i < 3; i++) {
          const Mesh &m = triad_meshes[i];
          GpuVec4 triad_col{triad_data[i].material.baseColor};
          command_buffer.pushFragmentUniform(0, &triad_col, sizeof(triad_col));
          rend::bindMesh(render_pass, m);
          rend::draw(render_pass, m);
        }
      }

      SDL_EndGPURenderPass(render_pass);
    } else {
      SDL_Log("Failed to acquire swapchain: %s", SDL_GetError());
      break;
    }

    guiSys.render(command_buffer);

    command_buffer.submit();

    if (performRecording) {
      record_callback();
    }
    frameNo++;
  }

  robotShapes.clear();
  SDL_ReleaseGPUGraphicsPipeline(device, meshPipeline);
  SDL_ReleaseGPUGraphicsPipeline(device, debugLinePipeline);

  guiSys.release();
  renderer.destroy();
  SDL_DestroyWindow(renderer.window);
  SDL_Quit();
  return 0;
}
