/**
Software License Agreement (proprietary)

\file      spinnaker_test_node.cpp
\authors   Teyvonia Thomas <tthomas@clearpathrobotics.com>
\copyright Copyright (c) 2017, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the express permission of Clearpath Robotics.
*/

// ROS Includes
#include "ros/ros.h"

// Spinnaker Includes
#include "Spinnaker.h"
// #include "SpinGenApi/SpinnakerGenApi.h"


namespace pointgrey_camera_driver
{

class SpinnakerTestNode
{

public:
  SpinnakerTestNode();

  void test();

};


SpinnakerTestNode::SpinnakerTestNode()
{
  test();
}


void SpinnakerTestNode::test()
{
  Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();

  Spinnaker::InterfaceList interfaceList = system->GetInterfaces();
  unsigned int numInterfaces = interfaceList.GetSize();
  std::printf("\033[93m[Spinnaker] Number of interfaces detected: %d \n",  numInterfaces);

  Spinnaker::CameraList camList = system->GetCameras();
  unsigned int numCameras = camList.GetSize();

  std::printf("\033[93m[Spinnaker] # of connected cameras: %d \n",  numCameras);

  // Finish if there are no cameras
  if (numCameras == 0)
  {
    std::printf("\033[91mNO Cameras Connected! \n\n");
    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    return;
  }
  else
  {
    std::printf("\033[92m[%d] Cameras Connected! \n\n", numCameras);
  }
}

}  // namespace pointgrey_camera_driver


int main(int argc, char** argv)
{
  ros::init(argc, argv, "spinnaker_test_node");
  pointgrey_camera_driver::SpinnakerTestNode node;
  ros::spin();
  return 0;
}
