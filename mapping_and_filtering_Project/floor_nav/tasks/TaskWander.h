#ifndef TASK_WANDER_H
#define TASK_WANDER_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskWanderConfig : public TaskConfig {
        TaskWanderConfig() {
            define("front_sector",  false,    "If true, use a front sector", true, front_sector);
            define("angular_range",  30*M_PI/180.,    "Size of angular sensors (in rad)", true, angular_range);
            define("safety_range",  0.25,    "Distance at which we need to stop", false, safety_range);
            define("dont_care_range",  1.0,    "Distance at which we can ignore obstacles", false, dont_care_range);
            define("multiplier",  1.0,    "Gain for the rotation control", false, multiplier);
            define("max_linear_speed",  1.0,    "Max allowed linear speed", false, max_linear_speed);
            define("max_angular_speed",  1.0,    "Max allowed angular speed", false, max_angular_speed);
        }
        bool front_sector;
        double multiplier;
        double angular_range;
        double safety_range;
        double dont_care_range;
        double max_linear_speed;
        double max_angular_speed;

    };
    class TaskWander : public TaskInstance<TaskWanderConfig,SimTasksEnv>
    {
        public:
            TaskWander(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWander() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryWander : public TaskDefinition<TaskWanderConfig, SimTasksEnv, TaskWander>
    {

        public:
            TaskFactoryWander(TaskEnvironmentPtr env) : 
                Parent("Wander","Wander aimlessly forever",true,env) {}
            virtual ~TaskFactoryWander() {};
    };
};

#endif // TASK_WANDER_H
