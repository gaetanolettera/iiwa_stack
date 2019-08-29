#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pivoting_actionlib/RLabinterpolationAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointPositionVelocity.h>


class RLabinterpolationAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pivoting_actionlib::RLabinterpolationAction> as_; // NodeHandle instance must be created before this line.
  std::string action_name_;
  // create messages that are used to published feedback/result
  pivoting_actionlib::RLabinterpolationResult result_;

public:

  RLabinterpolationAction(std::string name) :
    as_(nh_, name, boost::bind(&RLabinterpolationAction::executeInterpolation, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~RLabinterpolationAction(void)
  {
  }

  void executeInterpolation(const pivoting_actionlib::RLabinterpolationGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1000);
    bool success = true;

    // Obtaining the action-goal configuration parameters

    // ROS publisher
    ros::Publisher iiwa_pub = nh_.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);	

    // start executing the action
    



    if(success)
    {
      result_.interpolation_success = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rlabinterpolation_server");

  RLabinterpolationAction rlabinterpolation("rlabinterpolation");
  ros::spin();

  return 0;
}
