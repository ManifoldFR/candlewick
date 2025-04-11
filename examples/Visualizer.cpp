#include "candlewick/multibody/Visualizer.h"

#include <robot_descriptions_cpp/robot_load.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <chrono>

#include <CLI/App.hpp>
#include <CLI/Formatter.hpp>
#include <CLI/Config.hpp>

namespace cdw = candlewick;
using namespace candlewick::multibody;
using std::chrono::steady_clock;

int main(int argc, char **argv) {
  CLI::App app{"Visualizer example"};
  argv = app.ensure_utf8(argv);
  std::vector<Uint32> window_dims{1920u, 1080u};
  double fps = 50.0;
  app.add_option("--dims", window_dims, "Window dimensions.")
      ->capture_default_str();
  app.add_option<double, unsigned int>("--fps", fps, "Framerate")
      ->default_str("50.0");

  CLI11_PARSE(app, argc, argv);

  if (window_dims.size() != 2) {
    cdw::terminate_with_message("Expected only two values for argument --dims");
  }

  pin::Model model;
  pin::GeometryModel geom_model;
  robot_descriptions::loadModelsFromToml("ur.toml", "ur5_gripper", model,
                                         &geom_model, NULL);

  Visualizer visualizer{{window_dims[0], window_dims[1]}, model, geom_model};
  assert(!visualizer.hasExternalData());

  Eigen::VectorXd q0 = pin::neutral(model);
  Eigen::VectorXd q1 = pin::randomConfiguration(model);

  double dt = 1. / static_cast<double>(fps);
  std::chrono::duration<double> dt_ms{dt};
  static_assert(std::same_as<decltype(dt_ms.count()), double>);
  double t = 0.;

  while (!visualizer.shouldExit()) {
    const auto now = steady_clock::now();

    double alpha = std::sin(t);
    Eigen::VectorXd q = pin::interpolate(model, q0, q1, alpha);

    visualizer.display(q);
    std::this_thread::sleep_until(now + dt_ms);

    t += dt;
  }
}
