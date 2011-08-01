#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>

/*
   1   5   9   3
   2   6   0   4
   3   7   1   5
   4   8   2   6
*/
void writePose(const geometry_msgs::Pose& pose_msg,
        const std::string& file_name)
{
    std::ofstream out(file_name.c_str());
    if (!out.is_open())
    {
        ROS_ERROR("Cannot write to file %s", file_name.c_str());
        return;
    }
    tf::Pose pose;
    tf::poseMsgToTF(pose_msg, pose);
    btMatrix3x3 mat = pose.getBasis();
    btVector3 origin = pose.getOrigin();
    for (int col = 0; col < 4; ++col)
        for (int row = 0; row < 4 ++row)
            out << mat[row][col] << " ";
    out << std::endl;
    out.close();
}



void callback(const geometry_msgs::PoseStampedConstPtr& arm_pose_msg,
    const geometry_msgs::PoseStampedConstPtr& pattern_pose_msg)
{
    ROS_INFO("Received arm and pattern poses sync'ed");
    std::string arm_poses_file_name = ros::package::getPath(ROS_PACKAGE_NAME) + "/arm_poses.txt";
    writePose(arm_pose_msg->pose, arm_poses_file_name);
    std::string pattern_poses_file_name = ros::package::getPath(ROS_PACKAGE_NAME) + "/pattern_poses.txt";
    writePose(pattern_pose_msg->pose, pattern_poses_file_name);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronized_pose_recorder");

  ros::NodeHandle nh;
  message_filters::Subscriber<geometry_msgs::PoseStamped> arm_pose_sub(nh, "arm_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> pattern_pose_sub(nh, "pattern_pose", 1);

  ROS_INFO("Listening to synchronized topics %s and %s",
          nh.resolveName("arm_pose").c_str(),
          nh.resolveName("pattern_pose").c_str());

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), arm_pose_sub, pattern_pose_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}

