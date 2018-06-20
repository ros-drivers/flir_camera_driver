#ifndef _CM3_H_
#define _CM3_H_
#include "flir_camera_driver/camera.h"

namespace flir_camera_driver {
class Cm3 : public Camera
{
public:
  Cm3(Spinnaker::GenApi::INodeMap* node_map);
  ~Cm3();
  bool setFrameRate(const float frame_rate);
  bool setNewConfiguration(FlirConfig& config, const uint32_t& level);
};
}
#endif  // CM3_H
