#ifndef TASK_SET_HEADING_H
#define TASK_SET_HEADING_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskSetHeadingConfig : public TaskConfig {
        TaskSetHeadingConfig() {
            define("target",  0.,"Target angle",false, target);
            define("max_angular_velocity",  1.0,"Maximum angular velocity",false,max_angular_velocity);
            define("k_theta",  1.0,"Gain for angular control",false,k_theta);
            define("angle_threshold",  0.01,"Angular error at which the target is considered reached",false, angle_threshold);
            define("relative",  false,"Is the target pose relative or absolute",true, relative);
        }

        // convenience aliases, updated by update from the config data
        double target;
        double k_theta;
        double max_angular_velocity;
        double angle_threshold;
        bool relative;
    };

    class TaskSetHeading : public TaskInstance<TaskSetHeadingConfig,SimTasksEnv>
    {
        protected: 
            double initial_heading;
        public:
            TaskSetHeading(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetHeading() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactorySetHeading : public TaskDefinition<TaskSetHeadingConfig, SimTasksEnv, TaskSetHeading>
    {

        public:
            TaskFactorySetHeading(TaskEnvironmentPtr env) : 
                Parent("SetHeading","Reach a desired heading angle",true,env) {}
            virtual ~TaskFactorySetHeading() {};
    };
};

#endif // TASK_SET_HEADING_H
