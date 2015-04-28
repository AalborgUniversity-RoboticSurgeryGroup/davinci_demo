#include <string>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "forward_kinematics", ros::init_options::AnonymousName);
    KDL::Tree my_tree;
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
       ROS_ERROR("Failed to construct kdl tree");
       return -1;
    }

    for (KDL::SegmentMap::const_iterator it = my_tree.getSegments().begin(); it != my_tree.getSegments().end(); ++it)
    {
        std::cout << it->second.segment.getName() <<std::endl;
    }

    KDL::Chain my_chain;
    std::string root_link("base_link");
    std::string tip_link("needle_driver_jawbone_right");
    if (!(my_tree.getChain(root_link, tip_link, my_chain)))
    {
        ROS_ERROR("Failed to get chain");
        return -1;
    }

    // Create solver based on kinematic chain
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(my_chain);

    // Create joint array
    unsigned int nj = my_chain.getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);

    // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }

    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

    return 0;
}
