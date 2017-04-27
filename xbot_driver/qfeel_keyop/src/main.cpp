/**
 * License: BSD
 *   https:
 */

#include "qfeel_keyop/keyop.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "qfeel_keyop");
  qfeel_keyop::KeyOp keyop;
  if (keyop.init())
  {
    keyop.spin();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't initialise KeyOp!");
  }

  ROS_INFO_STREAM("Program exiting");
  return 0;
}
