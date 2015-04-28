#include <string>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "inverse_kinematics", ros::init_options::AnonymousName);
    ros::NodeHandle node;

    KDL::Tree my_tree;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
       ROS_ERROR("Failed to construct kdl tree");
       return -1;
    }

    KDL::Chain my_chain;
    std::string root_link("base_link");
    std::string tip_link("needle_driver_jawbone_right");
    if (!my_tree.getChain(root_link, tip_link, my_chain))
    {
        ROS_ERROR("Failed to get chain from tree");
        return -1;
    }

    for (unsigned int i = 0; i < my_chain.getNrOfSegments(); ++i)
    {
        std::cout << my_chain.getSegment(i).getName() << "(" << my_chain.getSegment(i).getJoint().getName() << ")" << std::endl;
    }

    //Create solver based on kinematic chain
    KDL::ChainFkSolverPos_recursive fksolver(my_chain);
    KDL::ChainIkSolverVel_pinv iksolverv(my_chain);
    KDL::ChainIkSolverPos_NR iksolver = KDL::ChainIkSolverPos_NR(my_chain,fksolver,iksolverv,100,1e-6);

    KDL::JntArray q(my_chain.getNrOfJoints());
    KDL::JntArray q_init(my_chain.getNrOfJoints());

    //Set destination frame
    double x, y, z;
    std::cout << "Set end-effector position <x y z>:" << std::endl;
    std::cin >> x >> y >> z;
    KDL::Vector dest_pos(x,y,z);
    KDL::Frame dest_frame(dest_pos);

    // Compute!
    int ret = iksolver.CartToJnt(q_init,dest_frame,q);

    for (unsigned int i = 0; i < q.rows(); ++i)
    {
        std::cout << "Joint #" << i << ": " << q(i) << std::endl;
    }

    return 0;
}
