/**
 * License: BSD
 *   https:
 */

#ifndef QFEEL_KEYOP_KEYOP_HPP_
#define QFEEL_KEYOP_KEYOP_HPP_

#include <ros/ros.h>
#include <termios.h> // for keyboard input
#include <ecl/threads.hpp>
#include <geometry_msgs/Twist.h>  // for velocity commands
#include <geometry_msgs/TwistStamped.h>  // for velocity commands
#include "qfeel_keyop/location.h"
namespace qfeel_keyop
{
  typedef uint32_t uint32;
/**
 * @brief Keyboard remote control for your robot
 *
 */
class KeyOp
{
public:
  /*********************
   ** C&D
   **********************/
  KeyOp();
  ~KeyOp();
  bool init();

  /*********************
   ** Runtime
   **********************/
  void spin();

private:
  ros::Publisher velocity_publisher_, enable_motors_publisher_, disable_motors_publisher_,up_down_publisher_;
  bool last_zero_vel_sent_;
  bool accept_incoming_;
  bool power_status_;
  bool wait_for_connection_;
  geometry_msgs::TwistPtr cmd_;
  geometry_msgs::TwistStampedPtr cmd_stamped_;
  double linear_vel_step_, linear_vel_max_;
  double angular_vel_step_, angular_vel_max_;
  uint32 up_down_location_,max_location_,min_location_,location_step_;
  qfeel_keyop::location location_;
  std::string name_;

  /*********************
   ** Commands
   **********************/
  void enable();
  void disable();
  void incrementLinearVelocity();
  void decrementLinearVelocity();
  void incrementAngularVelocity();
  void decrementAngularVelocity();
  void moveLeft();
  void moveRight();
  void pushUp();
  void pushDown();
  void resetVelocity();

  /*********************
   ** Keylogging
   **********************/

  void keyboardInputLoop();
  void processKeyboardInput(char c);
  void restoreTerminal();
  bool quit_requested_;
  int key_file_descriptor_;
  struct termios original_terminal_state_;
  ecl::Thread thread_;
};

} // namespace qfeel_keyop

#endif 
