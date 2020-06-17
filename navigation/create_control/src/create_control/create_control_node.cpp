#include "create_hw_interface.hpp"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "create_hw_interface");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh("");
  int rate;
  nh.param<int>("rate", rate, 100);

  Create2Interface robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate r(rate);
  while (ros::ok()) {
    robot.read();
    cm.update(robot.get_time(), robot.get_period());
    robot.write();
    r.sleep();
  }
}
