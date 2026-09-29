#ifndef TASK_PlanTo_H
#define TASK_PlanTo_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskPlanToConfig : public TaskConfig {
        TaskPlanToConfig() {
            define("goal_x",  0.,"X coordinate of destination",false, goal_x);
            define("goal_y",  0.,"Y coordinate of destination",false, goal_y);
            define("goal_theta",  0.,"Theta coordinate of destination",false, goal_theta);
            define("dist_threshold",  0.1,"Distance at which a the target is considered reached",false, dist_threshold);
            define("pub_period",  2.0,"Period at which a the target is republished",false, pub_period);
        }

        // convenience aliases, updated by update from the config data
        double goal_x,goal_y,goal_theta;
        double dist_threshold;
        double pub_period;

    };

    class TaskPlanTo : public TaskInstance<TaskPlanToConfig,SimTasksEnv>
    {
        protected:
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub;
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr trackSub;
            geometry_msgs::msg::Pose2D error;
            rclcpp::Time publish_time;
            void error_cb(geometry_msgs::msg::Pose2D::SharedPtr msg);
        public:
            TaskPlanTo(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskPlanTo() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();
    };
    class TaskFactoryPlanTo : public TaskDefinition<TaskPlanToConfig, SimTasksEnv, TaskPlanTo>
    {

        public:
            TaskFactoryPlanTo(TaskEnvironmentPtr env) : 
                Parent("PlanTo","Reach a desired destination using the path planner",true,env) {}
            virtual ~TaskFactoryPlanTo() {};
    };
};

#endif // TASK_PlanTo_H
