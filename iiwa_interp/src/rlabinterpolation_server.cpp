#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <iiwa_interp/RLabinterpolationAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iiwa_msgs/JointPosition.h>

#include "TooN/TooN.h"

#include "Traj_Generators/Quintic_Poly_Traj.h"
#include "Traj_Generators/Vector_Independent_Traj.h"

using namespace std;
using namespace TooN;


class RLabinterpolationAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<iiwa_interp::RLabinterpolationAction> as_; // NodeHandle instance must be created before this line.
  std::string action_name_;
  // create messages that are used to published feedback/result
  iiwa_interp::RLabinterpolationResult result_;
  iiwa_interp::RLabinterpolationFeedback feedback_;

  // ROS publisher
  ros::Publisher iiwa_pub;	

  double start_delay = 0.02;

public:

  RLabinterpolationAction(std::string name) :
    as_(nh_, name, boost::bind(&RLabinterpolationAction::executeInterpolation, this, _1), false),
    action_name_(name)
  {
    iiwa_pub = nh_.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);
    as_.start(); 
  }

  ~RLabinterpolationAction(void)
  {
  }

  void executeInterpolation(const iiwa_interp::RLabinterpolationGoal::ConstPtr& goal_msg)
  {
    // helper variables
    ros::Rate loop_rate(1000);
    bool success = true;

    std::shared_ptr<iiwa_interp::RLabinterpolationGoal> goal = std::make_shared<iiwa_interp::RLabinterpolationGoal>(*goal_msg);

    // Obtaining the action-goal configuration parameters

    /*std::vector<int> joint_order;
    for( auto joint_name : goal->planned_trajectory.joint_names ){
      if( joint_name == "iiwa_joint_1" ){
        joint_order.push_back(0);
      }
      else if( joint_name == "iiwa_joint_2" ){
        joint_order.push_back(1);
      }
      else if( joint_name == "iiwa_joint_3" ){
        joint_order.push_back(2);
      }
      else if( joint_name == "iiwa_joint_4" ){
        joint_order.push_back(3);
      }
      else if( joint_name == "iiwa_joint_5" ){
        joint_order.push_back(4);
      }
      else if( joint_name == "iiwa_joint_6" ){
        joint_order.push_back(5);
      }
      else if( joint_name == "iiwa_joint_7" ){
        joint_order.push_back(6);
      }
    }*/

    //Initial Time
    double t0 = ros::Time::now().toSec() + start_delay;

    //Scale Traj
    for( int i = 1; i<goal->planned_trajectory.points.size(); i++ )
    {
      double traj_scale = 1.5;
      goal->planned_trajectory.points[i].time_from_start = ros::Duration(goal->planned_trajectory.points[i].time_from_start.toSec()/traj_scale);
    }

    // contstruct traj
    std::vector<Vector_Independent_Traj> vect_trajs;
    for( int i = 1; i<goal->planned_trajectory.points.size(); i++ )
    {
      //Order vector

      Vector_Independent_Traj this_via_traj;
      for( int j = 0; j<7; j++ )
      {
        this_via_traj.push_back_traj(
              Quintic_Poly_Traj(   
                                  goal->planned_trajectory.points[i].time_from_start.toSec() - goal->planned_trajectory.points[i-1].time_from_start.toSec(), //duration
                                  goal->planned_trajectory.points[i-1].positions[ j ], //pi
                                  goal->planned_trajectory.points[i].positions[ j ], //pf
                                  t0 + goal->planned_trajectory.points[i-1].time_from_start.toSec(),//initial time                            
                                  goal->planned_trajectory.points[i-1].velocities[ j ], //vi
                                  goal->planned_trajectory.points[i].velocities[ j ], //vf
                                  goal->planned_trajectory.points[i-1].accelerations[ j ], //ai 
                                  goal->planned_trajectory.points[i].accelerations[ j ] //af
            )         
        );
      }

      vect_trajs.push_back(this_via_traj);

    }

    bool _b_action_preempted = false;
    double time_now = ros::Time::now().toSec();

    for( auto & traj : vect_trajs )
    {
      while( ros::ok() && !_b_action_preempted && !traj.isCompleate(time_now) )
      {

        ros::spinOnce();
        if (as_.isPreemptRequested()){
            _b_action_preempted = true;
            break;
        }

        time_now = ros::Time::now().toSec();

        Vector<> qR = traj.getPosition( time_now );

        publishQR(qR);

        publishFeedback( vect_trajs.back().getTimeLeft(time_now) ); //todo

        loop_rate.sleep();

      }
    }

    if( !ros::ok() || _b_action_preempted )
    {
      success = false;
    }

    result_.interpolation_success = true;
    ROS_INFO( "%s: Succeeded with ", action_name_.c_str());
    ROS_INFO( (success ? "TRUE" : "FALSE"));
    as_.setSucceeded(result_);

  }

  void publishQR( Vector<> qR ) {

    iiwa_msgs::JointPosition out_msg;

    out_msg.position.a1 = qR[0];
    out_msg.position.a2 = qR[1];
    out_msg.position.a3 = qR[2];
    out_msg.position.a4 = qR[3];
    out_msg.position.a5 = qR[4];
    out_msg.position.a6 = qR[5];
    out_msg.position.a7 = qR[6];

    iiwa_pub.publish(out_msg);

  }

  void publishFeedback( double time_left )
  {

    feedback_.time_left = time_left;

    as_.publishFeedback(feedback_);

  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rlabinterpolation_server");

  RLabinterpolationAction rlabinterpolation("rlabinterpolation");
  ros::spin();

  return 0;
}
