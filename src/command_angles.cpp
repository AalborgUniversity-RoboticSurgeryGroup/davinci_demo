#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "command_angles", ros::init_options::AnonymousName);
    ros::NodeHandle node;

    ros::Publisher setpoints_pub = node.advertise<trajectory_msgs::JointTrajectory>("command", 1);

    // Get joints
    std::vector<std::string> joints;
    node.param("/davinci/p4_hand_controller/joints", joints, std::vector<std::string>());

    std::cout << "Give me the setpoints:" << std::endl;
    double input;
    std::vector<double> setpoints;
    for (std::vector<std::string>::iterator i = joints.begin(); i != joints.end(); ++i)
    {
        std::cout << *i << std::endl;
        std::cin >> input;
        setpoints.push_back(input);
    }

    trajectory_msgs::JointTrajectory my_trajectory;
    my_trajectory.joint_names.assign(joints.begin(), joints.end());
    trajectory_msgs::JointTrajectoryPoint p;
    for (std::vector<double>::iterator s = setpoints.begin(); s != setpoints.end(); ++s)
    {
        p.positions.push_back(*s);
        p.velocities.push_back(0);
        p.accelerations.push_back(0);
        p.effort.push_back(0);
    }
    p.time_from_start = (ros::Duration(1));
    my_trajectory.points.push_back(p);

    setpoints_pub.publish(my_trajectory);

    return 0;
}
