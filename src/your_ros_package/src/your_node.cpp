#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "your_node_name");
  ros::NodeHandle nh;

  auto pub = nh.advertise<std_msgs::String>("your_topic_name", 1000);

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello from ROS! " << ros::Time::now().toSec();
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
