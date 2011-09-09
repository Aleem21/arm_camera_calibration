#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

#include <arm_control/MoveJointsAction.h>

#include <melfa/joint_state.h>
#include <melfa_ros/robot_path.h>
#include <melfa_ros/conversions.h>

class ArmCameraCalibration
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber arm_pose_sub_;
    ros::Subscriber pattern_pose_sub_;
    actionlib::SimpleActionClient<arm_control::MoveJointsAction>  action_client_;

    bool arm_pose_recorded_;
    bool pattern_pose_recorded_;

    std::queue<melfa::JointState> calibration_joint_states_;

    geometry_msgs::Pose last_arm_pose_;
    geometry_msgs::Pose last_pattern_pose_;

    ros::Time waypoint_reached_time_;
        
  public:

    ArmCameraCalibration() : nh_(), nh_private_("~"), 
        action_client_("robot_arm/move_joints_action_server")
    {
        ROS_INFO("Listening to topics %s and %s",
            nh_.resolveName("arm_pose").c_str(),
            nh_.resolveName("pattern_pose").c_str());
        arm_pose_sub_ = nh_.subscribe("arm_pose", 1, &ArmCameraCalibration::armPoseCallback, this);
        pattern_pose_sub_ = nh_.subscribe("pattern_pose", 1, &ArmCameraCalibration::patternPoseCallback, this);
    }

    void init(const std::queue<melfa::JointState>& joint_states)
    {
        calibration_joint_states_ = joint_states;
        ROS_INFO("Waiting for move arm action server...");
        action_client_.waitForServer();
        ROS_INFO("Server has been started.");
    }

    bool wayPointsLeft()
    {
        return (calibration_joint_states_.size() > 0);
    }

    void gotoNextWaypoint()
    {
        arm_control::MoveJointsGoal goal;
        melfa_ros::jointStateToJointStateMsg(calibration_joint_states_.front(), goal.target_joint_state);
        action_client_.sendGoal(goal);
        calibration_joint_states_.pop();
        ROS_INFO("Sent next joint way point. %zu way points left.", calibration_joint_states_.size());
        //wait for the action to return
        action_client_.waitForResult();
        actionlib::SimpleClientGoalState state = action_client_.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        sleep(1); // let the robot arm stay still a moment
        waypoint_reached_time_ = ros::Time::now();
        ROS_DEBUG_STREAM("Waypoint reached at " << waypoint_reached_time_);
    }

    void recordPoses()
    {
        ROS_INFO("Waiting for pose messages...");
        arm_pose_recorded_ = false;
        pattern_pose_recorded_ = false;
        while (!arm_pose_recorded_ || !pattern_pose_recorded_ )
        {
            ros::spinOnce();
            if (!ros::ok()) return;
            if (!arm_pose_recorded_) ROS_DEBUG("Arm pose still missing");
            if (!pattern_pose_recorded_) ROS_DEBUG("Pattern pose still missing");
        }
        ROS_DEBUG("Both poses received.");
        ROS_INFO("Saving poses.");
        std::string arm_poses_file_name = ros::package::getPath(ROS_PACKAGE_NAME) + "/arm_poses.txt";
        writePose(last_arm_pose_, arm_poses_file_name);
        std::string pattern_poses_file_name = ros::package::getPath(ROS_PACKAGE_NAME) + "/pattern_poses.txt";
        writePose(last_pattern_pose_, pattern_poses_file_name);
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
        std::ofstream out(file_name.c_str(), std::ios_base::app);
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

    void armPoseCallback(const geometry_msgs::PoseStampedConstPtr& arm_pose_msg)
    {
        ROS_DEBUG_STREAM("Received arm pose message with time " << arm_pose_msg->header.stamp);
        if (arm_pose_msg->header.stamp > waypoint_reached_time_)
        {
            last_arm_pose_ = arm_pose_msg->pose;
            arm_pose_recorded_ = true;
        }
        else
        {
            ROS_DEBUG_STREAM("Arm pose message too old, waiting for next.");
        }
    }

    void patternPoseCallback(const geometry_msgs::PoseStampedConstPtr& pattern_pose_msg)
    {
        ROS_DEBUG_STREAM("Received pattern pose message with time " << pattern_pose_msg->header.stamp);
        if (pattern_pose_msg->header.stamp > waypoint_reached_time_)
        {
            last_pattern_pose_ = pattern_pose_msg->pose;
            pattern_pose_recorded_ = true;
        }
        else
        {
            ROS_DEBUG_STREAM("Pattern pose message too old, waiting for next.");
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

    std::queue<melfa::JointState> calibration_joint_states = melfa_ros::readJointPath(argv[1]);

    ArmCameraCalibration calibration;
    calibration.init(calibration_joint_states);
    while (calibration.wayPointsLeft() && ros::ok())
    {
        calibration.gotoNextWaypoint();
        calibration.recordPoses();
    }
    ROS_INFO("All calibration poses reached and poses recorded.");

    return 0;
}

