#pragma once

#include "Multibody.h"
#include <pinocchio/visualizers/base-visualizer.hpp>
#include <zmq.h>

namespace candlewick {
namespace multibody {
  class VisualizerClient final : public pin::visualizers::BaseVisualizer {
  public:
  };
} // namespace multibody
} // namespace candlewick
