#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <vector>

#include <sstream>

struct Joint
{
  std::string name;
  double position;
  double max_position;

  Joint(std::string name, double max_position, double position): 
  name(name), max_position(max_position), position(position)
  {
  }

  Joint(std::string name, double max_position):Joint(name, max_position, 0.0){}

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agrolab_controller");
  ros::NodeHandle n;

  ros::Publisher agrolab_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  ros::Rate loop_rate(2);

  // Создаем массив сочленений
  std::vector<Joint> joints = {
    Joint("base_link_to_X", 0.43),
    Joint("X_to_Y", 0.54),
    Joint("Y_to_Z", 0.2)
    };

  double pos = 0.0; // положение сочленения
  double pos_incr = 0.01; // шаг движения сочленения
  int j_index = 0; // индекс сочленения, которое будем двигать

  // Массив имен сочленений
  std::vector<std::string> names;
  std::transform(joints.begin(), joints.end(), std::back_inserter(names), 
                [](Joint j){return j.name;});

  // Массив положений сочленений
  std::vector<double> positions(joints.size(), 0.0);

  // Создаем сообщение
  sensor_msgs::JointState state_msg;
  state_msg.name = names;

  while (ros::ok())
  {
    state_msg.header.stamp = ros::Time::now();
    state_msg.position = positions;

    // Публикуем сообшение
    agrolab_pub.publish(state_msg);
    
    ROS_INFO("Published positions: %0.2f, %0.2f, %0.2f", positions[0], positions[1], positions[2]);
    ros::spinOnce();

    loop_rate.sleep();

    // Двигаем сочленение
    positions[j_index] = pos;
    pos += pos_incr;
    if(pos >= joints[j_index].max_position || pos <= 0.0)
    {
      pos_incr *= -1;
    }

  }

  return 0;
}