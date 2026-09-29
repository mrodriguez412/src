#include <math.h>
#include "TaskPlanTo.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_PlanTo
#ifdef DEBUG_PlanTo
#warning Debugging task PlanTo
#endif

using std::placeholders::_1;

TaskIndicator TaskPlanTo::initialise() 
{
    error.x = NAN;
    publish_time = getNode()->get_clock()->now();
    RCLCPP_INFO(getNode()->get_logger(),"Going to %.2f %.2f",cfg->goal_x,cfg->goal_y);
    goalPub = getNode()->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal",1);
    trackSub = getNode()->create_subscription<geometry_msgs::msg::Pose2D>("/path_follower/error",1,
            std::bind(&TaskPlanTo::error_cb, this, std::placeholders::_1));
    return TaskStatus::TASK_INITIALISED;
}

void TaskPlanTo::error_cb(geometry_msgs::msg::Pose2D::SharedPtr msg) {
    error = *msg;
}


TaskIndicator TaskPlanTo::iterate()
{
    rclcpp::Time now  = getNode()->get_clock()->now();

    if ((now - publish_time).seconds() > cfg->pub_period) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = env->getReferenceFrame();
        msg.pose.position.x = cfg->goal_x;
        msg.pose.position.y = cfg->goal_y;
        tf2::Quaternion q; q.setRPY(0,0,cfg->goal_theta);
        msg.pose.orientation = tf2::toMsg(q);
        goalPub->publish(msg);
        publish_time = now;
    }

    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
    double r = hypot(cfg->goal_y-tpose.y,cfg->goal_x-tpose.x);
    if ((r < cfg->dist_threshold) && (error.x < cfg->dist_threshold)) {
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}


DYNAMIC_TASK(TaskFactoryPlanTo);
