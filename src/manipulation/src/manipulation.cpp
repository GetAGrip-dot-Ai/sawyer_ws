#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

geometry_msgs::Pose poi_pose;
geometry_msgs::Pose basket_pose;
geometry_msgs::Pose reset_pose;

geometry_msgs::Pose box_pose;
shape_msgs::SolidPrimitive box_primitive;


// create obstacle
void make_obstacle_callback(const manipulation::obstacle_msg msg)
{
	box_primitive = msg.primitive;
    box_pose = msg.pose;
    // create_scene_object()
}


// update POI
void update_POI_callback(const geometry_msgs::Pose msg)
{
	poi = msg;
}


// harvest service callback
bool harvest_srv_callback(manipulation_node::harvest::Request& request, manipulation_node::harvest::Response& response)
{

    int state = request.req_id;
    // get state
    if(state==0){ // move to poi
        // commander moveToPoseWithConstraint(poi_pose)
        return true;
    }
    if(state==1){ // move to basket
        // commander moveToPose(basket_pose)
        return true;
    }
    if(state==2){ //reset
        // commander moveToPose(reset_pose)
        return true;
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "manipulation_node");
    ros::NodeHandle n;

    ros::Subscriber obstacle_sub = n.subscribe("obstacle_pose", 1000, make_obstacle_callback);
    ros::Subscriber poi_sub = n.subscribe("poi_pose", 1000, update_POI_callback);
    ros::ServiceServer service = n.advertiseService("harvest",harvest_srv_callback);

    ros::Rate loop_rate(20);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
    
}

