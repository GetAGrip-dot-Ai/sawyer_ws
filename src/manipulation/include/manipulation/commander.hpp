// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>


// const double tau = 2 * M_PI;
// static const std::string PLANNING_GROUP = "right_arm";
// namespace rvt = rviz_visual_tools;

// class Commander
// {
  // ros::NodeHandle nh_;

// public:
  
  // Commander()
  // {

  // }

  // ~Commander()
  // {
  //   // cv::destroyWindow(OPENCV_WINDOW);
  // }

  // void setTagLocations(float x_det, float y_det, float z_det)
  // {
	//   //TODO: Update tag locations
  // }

  // void imageCb(const sensor_msgs::ImageConstPtr& msg)
  // {
  //   cv_bridge::CvImagePtr cv_ptr;
  //   try
  //   {
  //     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  //   }
  //   catch (cv_bridge::Exception& e)
  //   {
  //     ROS_ERROR("cv_bridge exception: %s", e.what());
  //     return;
  //   }

	// //TODO: Draw circles at tag locations on image. 

  //   // Update GUI Window
  //   cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //   cv::waitKey(3);

  //   // TODO:Output modified video stream
  //   // Convert the modified frames into sensor_msgs::Image message and publish it using image_pub
  // }

// private:
//   float x_loc ,y_loc;
//   std::vector<float> x_arr;
//   std::vector<float> y_arr;
// };
