#ifndef TASK_AUTODOCK_H
#define TASK_AUTODOCK_H

#include "task_manager_lib/TaskInstance.h"
#include "task_manager_lib/TaskActionGeneric.h"
#include "kobuki_ros_interfaces/action/auto_docking.hpp"
#include "floor_nav/SimTasksEnv.h"

using namespace task_manager_lib;

namespace floor_nav {
    using AutoDockingAction = kobuki_ros_interfaces::action::AutoDocking;
    struct TaskAutoDockConfig : public task_manager_lib::TaskActionGenericConfig {
        TaskAutoDockConfig() : task_manager_lib::TaskActionGenericConfig("/auto_docking_action") {
        }

    };

    class TaskAutoDock : public TaskActionGeneric<AutoDockingAction,TaskAutoDockConfig, SimTasksEnv>
    {
        protected:
            //
            typedef TaskActionGeneric<AutoDockingAction,
                    TaskAutoDockConfig, SimTasksEnv> Parent;

            virtual void buildActionGoal(typename Parent::Goal & goal) {
                // This goal is empty
            }
        public:
            TaskAutoDock(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskAutoDock() {};
    };
    class TaskFactoryAutoDock : public TaskDefinition<TaskAutoDockConfig, SimTasksEnv, TaskAutoDock>
    {

        public:
            TaskFactoryAutoDock(TaskEnvironmentPtr env) : 
                Parent("AutoDock","Call the AutoDocking action",true,env) {}
            virtual ~TaskFactoryAutoDock() {};
    };
};

#endif // TASK_AUTODOCK_H
