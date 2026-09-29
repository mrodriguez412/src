#ifndef TASK_SET_BOOL_H
#define TASK_SET_BOOL_H

#include "task_manager_lib/TaskServiceGeneric.h"
#include "floor_nav/SimTasksEnv.h"
#include "std_srvs/srv/set_bool.hpp"
using namespace task_manager_lib;

namespace floor_nav {
    struct TaskSetBoolConfig : public TaskServiceGenericConfig {
        TaskSetBoolConfig() : TaskServiceGenericConfig("/service_name") {
            define("value",false,"value of the boolean",true);
        }
    };

    class TaskSetBool : public TaskServiceGeneric<std_srvs::srv::SetBool,TaskSetBoolConfig,SimTasksEnv>
    {
        protected:
            virtual void buildServiceRequest(Request& req) {
                req.data=cfg->get<bool>("value");
                RCLCPP_INFO(node->get_logger(),"Setting bool to %s",
                        req.data?"true":"false");
            }

        public:
            TaskSetBool(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetBool() {};
    };

    class TaskFactorySetBool : public TaskDefinition<TaskSetBoolConfig, SimTasksEnv, TaskSetBool>
    {

        public:
            TaskFactorySetBool(TaskEnvironmentPtr env) : 
                Parent("SetBool","Use a service to set a bool",true,env) {}
            virtual ~TaskFactorySetBool() {};
    };
}

#endif // TASK_SET_BOOL_H
