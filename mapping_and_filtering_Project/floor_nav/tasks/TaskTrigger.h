#ifndef TASK_Trigger_H
#define TASK_Trigger_H

#include "task_manager_lib/TaskServiceGeneric.h"
#include "floor_nav/SimTasksEnv.h"
#include "std_srvs/srv/trigger.hpp"
using namespace task_manager_lib;

namespace floor_nav {
    struct TaskTriggerConfig : public TaskServiceGenericConfig {
        TaskTriggerConfig() : TaskServiceGenericConfig("/service_name") {
        }
    };

    class TaskTrigger : public TaskServiceGeneric<std_srvs::srv::Trigger,TaskTriggerConfig,SimTasksEnv>
    {
        protected:

        public:
            TaskTrigger(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskTrigger() {};
    };

    class TaskFactoryTrigger : public TaskDefinition<TaskTriggerConfig, SimTasksEnv, TaskTrigger>
    {

        public:
            TaskFactoryTrigger(TaskEnvironmentPtr env) : 
                Parent("Trigger","Use a service to send a trigger",true,env) {}
            virtual ~TaskFactoryTrigger() {};
    };
}

#endif // TASK_Trigger_H
