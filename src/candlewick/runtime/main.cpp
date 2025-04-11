#include <zmq.hpp>
#include <zmq_addon.hpp>
#include "candlewick/multibody/Multibody.h"
#include "candlewick/multibody/Visualizer.h"

#include <pinocchio/serialization/model.hpp>
#include <pinocchio/serialization/geometry.hpp>

#include <CLI/App.hpp>
#include <CLI/Formatter.hpp>
#include <CLI/Config.hpp>

namespace cdw = candlewick;
namespace pin = pinocchio;
using cdw::multibody::Visualizer;

int main(int argc, char **argv) {
  CLI::App app{"Candlewick visualizer runtime"};
  argv = app.ensure_utf8(argv);

  std::vector<Uint32> window_dims{1920u, 1080u};
  app.add_option("--dims", window_dims, "Window dimensions.")
      ->capture_default_str();

  CLI11_PARSE(app, argc, argv);

  if (window_dims.size() != 2) {
    cdw::terminate_with_message("Expected only two values for argument --dims");
  }

  zmq::context_t ctx;
  zmq::socket_t sock{ctx, zmq::socket_type::pull};
  sock.bind("tcp://127.0.0.1:12000");
  const std::string endpoint = sock.get(zmq::sockopt::last_endpoint);
  printf("ZMQ endpoint: %s\n", endpoint.c_str());

  pin::Model model;
  pin::GeometryModel geom_model;

  std::vector<zmq::message_t> recv_models;

  printf("Receiving Model and GeometryModel...\n");

  const auto ret = zmq::recv_multipart(sock, std::back_inserter(recv_models));
  assert(*ret == 2);
  printf("Received %zu messages\n", ret.value());

  model.loadFromString(recv_models[0].to_string());
  geom_model.loadFromString(recv_models[1].to_string());

  Visualizer::Config config;
  config.width = window_dims[0];
  config.height = window_dims[1];
  Visualizer viz{config, model, geom_model};

  Eigen::VectorXd q0 = pin::neutral(model);

  while (!viz.shouldExit()) {
    viz.display(q0);
  }

  return 0;
}
