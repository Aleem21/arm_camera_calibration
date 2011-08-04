#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

#include <arm_control/MoveArmAction.h>

#include <melfa/robot_pose.h>
#include <melfa_ros/robot_path.h>

class ArmCameraCalibration
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    message_filters::Subscriber<geometry_msgs::PoseStamped> arm_pose_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pattern_pose_sub_;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> poses_synchronizer_;
    actionlib::SimpleActionClient<arm_control::MoveArmAction>  action_client_;

    bool waypoint_reached_;

    std::vector<melfa::RobotPose> calibration_path_;
        
  public:

    ArmCameraCalibration() : nh_(), nh_private_("~"), 
        arm_pose_sub_(nh_, "arm_pose", 1),
        pattern_pose_sub_(nh_, "pattern_pose", 1),
        poses_synchronizer_(MySyncPolicy(10), arm_pose_sub_, pattern_pose_sub_),
        action_client_("robot_arm/arm_control_action_server"), 
        waypoint_reached_(false)
    {
        // subscribe to synchronized pose topics
        poses_synchronizer_.registerCallback(boost::bind(&ArmCameraCalibration::posesCallback, this, _1, _2));
        ROS_INFO("Listening to synchronized topics %s and %s",
            nh_.resolveName("arm_pose").c_str(),
            nh_.resolveName("pattern_pose").c_str());
    }

    void init(const std::vector<melfa::RobotPose>& calibration_path)
    {
        calibration_path_ = calibration_path;

        ROS_INFO("Waiting for move arm action server...");
        action_client_.waitForServer();
        ROS_INFO("Server has been started.");
        sendNextWaypoint();
    }

    void sendNextWaypoint()
    {
        arm_control::MoveArmGoal goal;
        action_client_.sendGoal(goal);
    }

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
        for (int col = 0; col < 3; ++col)
        {
            for (int row = 0; row < 3; ++row)
                out << mat[row][col] << " ";
            out << "0" << " ";
        }
        for (int i = 0; i < 3; ++i)
            out << origin[i] << " ";
        out << "1" << std::endl;
        out.close();
    }


    void posesCallback(const geometry_msgs::PoseStampedConstPtr& arm_pose_msg,
        const geometry_msgs::PoseStampedConstPtr& pattern_pose_msg)
    {
        ROS_INFO("Received arm and pattern poses sync'ed");
        if (waypoint_reached_)
        {
            ROS_INFO("Waypoint reached, saving poses.");
            std::string arm_poses_file_name = ros::package::getPath(ROS_PACKAGE_NAME) + "/arm_poses.txt";
            writePose(arm_pose_msg->pose, arm_poses_file_name);
            std::string pattern_poses_file_name = ros::package::getPath(ROS_PACKAGE_NAME) + "/pattern_poses.txt";
            writePose(pattern_pose_msg->pose, pattern_poses_file_name);
            waypoint_reached_ = false;
            sendNextWaypoint();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_camera_calibration");
    if (argc < 2)
    {
        std::cerr << "USAGE: " << argv[0] << " <calibration poses config file>" << std::endl;
        exit(1);
    }

    std::vector<melfa::RobotPose> calibration_poses = melfa_ros::readRobotPath(argv[1]);

    ArmCameraCalibration calibration;
    calibration.init(calibration_poses);

    ros::spin();

    return 0;
}

