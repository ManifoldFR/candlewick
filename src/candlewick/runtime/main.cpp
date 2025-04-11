#include <zmq.hpp>
#include <zmq_addon.hpp>
#include "candlewick/multibody/Multibody.h"

#include <pinocchio/serialization/model.hpp>
#include <pinocchio/serialization/geometry.hpp>

namespace pin = pinocchio;

int main() {
  zmq::context_t ctx;
  zmq::socket_t sock{ctx, zmq::socket_type::pull};
  sock.bind("tcp://127.0.0.1:12000");
  const std::string endpoint = sock.get(zmq::sockopt::last_endpoint);
  printf("ZMQ endpoint: %s\n", endpoint.c_str());

  pin::Model model;
  pin::GeometryModel geom_model;

  std::vector<zmq::message_t> recv_models;

  printf("Receiving Model and GeometryModel...");

  const auto ret = zmq::recv_multipart(sock, std::back_inserter(recv_models));
  assert(*ret == 2);

  model.loadFromString(recv_models[0].to_string());
  std::cout << model << std::endl;
  geom_model.loadFromString(recv_models[1].to_string());

  return 0;
}
