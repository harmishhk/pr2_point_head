/*/
 * Copyright (c) 2016 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Fri Jul 15 2016
 */

// parameters that can also be set using rosparam
#define POINT_HEAD_TOPIC "point_head"
#define POINT_HEAD_CLIENT_NAME "/head_traj_controller/point_head_action"
#define POINTING_FRAME "head_plate_frame"

// other parameters
#define NODE_NAME "pr2_point_head"
#define SUBSCRIBERS_QUEUE_SIZE 5
#define PUBLISH_TIMER_FREQUENCY 5.0 // hz
#define MESSAGE_IGNORE_TIME 5 // sec, point-head goals older than this time will be discarded
#define MESSAGE_THROTTLE_TIME 10
#define MIN_DURATION 0.4 // minimum duration to reach pointing goal
#define MAX_VELOCITY 1.0 // rad/sec

#include <pr2_point_head/pr2_point_head.h>

PR2PointHead::PR2PointHead() {}

PR2PointHead::~PR2PointHead() {
  delete point_head_client_;
}

void PR2PointHead::init() {
  // get private node handle
  ros::NodeHandle nh("~/");

  // get parameters
  nh.param("point_head_topic", point_head_topic_, std::string(POINT_HEAD_TOPIC));
  nh.param("point_head_client_name", point_head_client_name_, std::string(POINT_HEAD_CLIENT_NAME));
  nh.param("pointing_frame", pointing_frame_, std::string(POINTING_FRAME));
  nh.param("min_duration", min_duration_, MIN_DURATION);
  nh.param("max_velocity", max_velocity_, MAX_VELOCITY);

  // initialize the client for the action interface to the head controller
  point_head_client_ = new PointHeadClient(point_head_client_name_, true);

  // wait for head controller action server to come up
  while(!point_head_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("waiting for the point_head_action server to come up");
  }

  // set initial current_point_head_goal to front of the robot
  current_point_head_goal.header.stamp = ros::Time::now();
  current_point_head_goal.header.frame_id = "base_link";
  current_point_head_goal.point.x = 1.0;
  current_point_head_goal.point.z = 1.2;

  // setup timers
  publish_timer = nh.createTimer(ros::Duration(1 / PUBLISH_TIMER_FREQUENCY),
    &PR2PointHead::publishPointingCommand, this);

  // subscribe to head pointing orders
  point_head_subscriber_ = nh.subscribe(point_head_topic_, SUBSCRIBERS_QUEUE_SIZE,
    &PR2PointHead::newPointHeadCmdReceived, this);
}

// callback for point-head messages
void PR2PointHead::newPointHeadCmdReceived(const geometry_msgs::PointStamped& msg)
{
    ROS_DEBUG_NAMED(NODE_NAME, "%s: got point head goal, x=%f, y=%f, z=%f, frame=%s",
      NODE_NAME, msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id.c_str());
    current_phg_mutex.lock();
    current_point_head_goal = msg;
    current_phg_mutex.unlock();
}

// call back to order poiting command
void PR2PointHead::publishPointingCommand(const ros::TimerEvent& event) {
  auto now = ros::Time::now();
  auto last_msg_time_diff = (now - current_point_head_goal.header.stamp).toSec();
  if (last_msg_time_diff > MESSAGE_IGNORE_TIME)
  {
      ROS_DEBUG_THROTTLE_NAMED(MESSAGE_THROTTLE_TIME, NODE_NAME,
          "%s: last point head goal was received before %f seconds",
          NODE_NAME, last_msg_time_diff);
      return;
  }

  // ignore timestamp in the goal and change it to now
  current_phg_mutex.lock();
  geometry_msgs::PointStamped current_point_head_goal_now;
  current_point_head_goal_now.header.frame_id = current_point_head_goal.header.frame_id;
  current_point_head_goal_now.header.stamp = now;
  current_point_head_goal_now.point = current_point_head_goal.point;
  current_phg_mutex.unlock();

  // set the goal message we will be sending
  pr2_controllers_msgs::PointHeadGoal goal;

  // we are pointing the high-def camera frame
  // (pointing_axis defaults to X-axis)
  goal.pointing_frame = pointing_frame_;
  goal.pointing_axis.x = 1;
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 0;
  goal.target = current_point_head_goal_now;

  // take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(min_duration_);

  // and go no faster than 1 rad/s
  goal.max_velocity = max_velocity_;

  // send the goal
  //point_head_client_->cancelAllGoals(); // previous goal is automatically cancelled in controller
  point_head_client_->sendGoal(goal);

  // wait for it to get there (abort after 2 secs to prevent getting stuck)
  //point_head_client_->waitForResult(ros::Duration(2));
}


// the main method starts a rosnode and initializes the Pr2PointHead class
int main(int argc, char **argv)
{
    // starting the pr2_point_head node
    ros::init(argc, argv, NODE_NAME);
    ROS_DEBUG_NAMED(NODE_NAME, "started %s node", NODE_NAME);

    // initiazling Pr2PointHead class
    PR2PointHead pr2PointHead;
    pr2PointHead.init();

    // start spinning the node
    ros::spin();

    return 0;
}
