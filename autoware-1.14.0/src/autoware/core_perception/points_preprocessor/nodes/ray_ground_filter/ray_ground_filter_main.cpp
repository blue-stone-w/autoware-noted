/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 ********************
 * amc-nu (abrahammonrroy@yahoo.com)
 */

#include <ros/ros.h>
#include "points_preprocessor/ray_ground_filter/ray_ground_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ray_ground_filter");
  RayGroundFilter app;

  app.Run();

  return 0;
}
