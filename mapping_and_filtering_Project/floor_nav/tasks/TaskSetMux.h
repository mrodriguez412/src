#ifndef TASK_SetMux_H
#define TASK_SetMux_H

#include "task_manager_lib/TaskServiceGeneric.h"
#include "floor_nav/SimTasksEnv.h"
#include "topic_tools_interfaces/srv/mux_select.hpp"
using namespace task_manager_lib;

namespace floor_nav {
    struct TaskSetMuxConfig : public TaskServiceGenericConfig {
        TaskSetMuxConfig() : TaskServiceGenericConfig("/mux/select") {
            define("topic","","name of the topic to select",true);
        }
    };

    class TaskSetMux : public TaskServiceGeneric<topic_tools_interfaces::srv::MuxSelect,TaskSetMuxConfig,SimTasksEnv>
    {
        protected:
            virtual void buildServiceRequest(Request& req) {
                req.topic=cfg->get<std::string>("topic");
                RCLCPP_INFO(node->get_logger(),"Selecting topic %s",
                        req.topic.c_str());
            }

        public:
            TaskSetMux(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetMux() {};
    };

    class TaskFactorySetMux : public TaskDefinition<TaskSetMuxConfig, SimTasksEnv, TaskSetMux>
    {

        public:
            TaskFactorySetMux(TaskEnvironmentPtr env) : 
                Parent("SetMuxGeneric","Control a mux",true,env) {}
            virtual ~TaskFactorySetMux() {};
    };
}

#endif // TASK_SetMux_H
