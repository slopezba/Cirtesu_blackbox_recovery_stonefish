#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cola2_msgs/String.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>

struct PredefinedPose
{
  trajectory_msgs::JointTrajectoryPoint point;
  std::vector<std::string> joint_names;
};

void request(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac, const PredefinedPose &pp)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = pp.joint_names;
  goal.trajectory.points.push_back(pp.point);
  goal.trajectory.points[0].time_from_start.sec = 10;

  goal.path_tolerance.resize(goal.trajectory.joint_names.size());
  goal.goal_tolerance.resize(goal.trajectory.joint_names.size());
  for (uint i = 0; i < pp.joint_names.size(); i++)
  {
    goal.path_tolerance[i].name = goal.trajectory.joint_names[i];
    goal.path_tolerance[i].position = 1.01;
    goal.path_tolerance[i].velocity = 1.0;
    goal.path_tolerance[i].acceleration = 1.0;

    goal.goal_tolerance[i].name = goal.trajectory.joint_names[i];
    goal.goal_tolerance[i].position = 1.0;
    goal.goal_tolerance[i].velocity = 1.0;
    goal.goal_tolerance[i].acceleration = 1.0;
  }

  ac.sendGoal(goal);
}

void request(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac,
             const control_msgs::FollowJointTrajectoryGoal &goal)
{
  ac.sendGoal(goal);
}

class predefined_positions
{
private:
  ros::NodeHandle nh_priv;

  std::map<std::string, PredefinedPose> predefined_poses;
  std::map<std::string, control_msgs::FollowJointTrajectoryGoal> predefined_trajectories;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac;
  ros::ServiceServer service_pp, service_pt;

  bool pp_callback(cola2_msgs::String::Request &req, cola2_msgs::String::Response &res);
  bool pt_callback(cola2_msgs::String::Request &req, cola2_msgs::String::Response &res);

public:
  predefined_positions();
};

predefined_positions::predefined_positions(/* args */)
    : ac("joint_trajectory_controller/follow_joint_trajectory", true), nh_priv("~")
{
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); // will wait for infinite time

  service_pp = nh_priv.advertiseService("goto", &predefined_positions::pp_callback, this);
  service_pt = nh_priv.advertiseService("do", &predefined_positions::pt_callback, this);
  ROS_INFO("Ready to get goto requests!");
}

bool predefined_positions::pp_callback(cola2_msgs::String::Request &req, cola2_msgs::String::Response &res)
{
  auto element = predefined_poses.find(req.data);
  if (element == predefined_poses.end())
  {
    ROS_INFO("Predefined pose not found in database. Gonna check param server");

    if (!nh_priv.hasParam("predefined_positions/" + req.data + "/positions"))
    {
      ROS_WARN("Predefined pose not found in param server either.");
      res.success = 0;
      res.message = "Predefined pose not loaded";
      return false;
    }

    PredefinedPose pp;

    nh_priv.getParam("predefined_positions/" + req.data + "/joint_names", pp.joint_names);
    nh_priv.getParam("predefined_positions/" + req.data + "/positions", pp.point.positions);

    predefined_poses.insert(std::make_pair(req.data, pp));

    request(ac, pp);
  }
  else
  {
    request(ac, element->second);
  }
  // wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(67.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    res.success = 1;
    return true;
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
    res.message = "Action did not finish before the time out";
    res.success = 0;
    return false;
  }
}

bool predefined_positions::pt_callback(cola2_msgs::String::Request &req, cola2_msgs::String::Response &res)
{
  auto element = predefined_trajectories.find(req.data);
  if (element == predefined_trajectories.end())
  {
    ROS_INFO("Predefined trajectory not found in database. Gonna check param server");

    if (!nh_priv.hasParam("predefined_trajectories/" + req.data + "/points"))
    {
      ROS_WARN("Predefined trajectory not found in param server either.");
      res.success = 0;
      res.message = "Predefined trajectory not loaded";
      return false;
    }

    control_msgs::FollowJointTrajectoryGoal g;

    nh_priv.getParam("predefined_trajectories/" + req.data + "/joint_names", g.trajectory.joint_names);

    XmlRpc::XmlRpcValue v;

    nh_priv.getParam("predefined_trajectories/" + req.data + "/points", v);

    g.trajectory.points.resize(v.size());
    for (uint i = 0; i < v.size(); i++)
    {
      XmlRpc::XmlRpcValue vp = v[i]["positions"];
      XmlRpc::XmlRpcValue vv = v[i]["velocities"];
      // XmlRpc::XmlRpcValue va = v[i]["accelerations"];
      g.trajectory.points[i].positions.resize(vp.size());
      g.trajectory.points[i].velocities.resize(vp.size());
      // g.trajectory.points[i].accelerations.resize(vp.size());
      for (uint j = 0; j < vp.size(); j++)
      {
        g.trajectory.points[i].positions[j] = static_cast<double>(vp[j]);
        g.trajectory.points[i].velocities[j] = static_cast<double>(vv[j]);
        // g.trajectory.points[i].accelerations[j] = static_cast<double>(va[j]);
      }

      g.trajectory.points[i].time_from_start.sec = static_cast<double>(v[i]["time_from_start"]["secs"]);
      g.trajectory.points[i].time_from_start.nsec = static_cast<double>(v[i]["time_from_start"]["nsecs"]);
    }

    g.path_tolerance.resize(g.trajectory.joint_names.size());
    g.goal_tolerance.resize(g.trajectory.joint_names.size());
    for (uint i = 0; i < g.trajectory.joint_names.size(); i++)
    {
      g.path_tolerance[i].name = g.trajectory.joint_names[i];
      g.path_tolerance[i].position = 0.1;
      g.path_tolerance[i].velocity = 1.0;
      g.path_tolerance[i].acceleration = 1.0;

      g.goal_tolerance[i].name = g.trajectory.joint_names[i];
      g.goal_tolerance[i].position = 0.1;
      g.goal_tolerance[i].velocity = 1.0;
      g.goal_tolerance[i].acceleration = 1.0;
    }

    predefined_trajectories.insert(std::make_pair(req.data, g));

    request(ac, g);
  }
  else
  {
    request(ac, element->second);
  }
  // wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(67.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    res.success = 1;
    return true;
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
    res.message = "Action did not finish before the time out";
    res.success = 0;
    return false;
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "predefined_positions");

  predefined_positions server;

  // exit
  ros::spin();
}
