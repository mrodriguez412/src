#ifndef TASK_WAIT_FOR_ROI_H
#define TASK_WAIT_FOR_ROI_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskWaitForROIConfig : public TaskConfig {
        TaskWaitForROIConfig() {
            define("roi_x",  0.,"X coordinate of the ROI",false, roi_x);
            define("roi_y",  0.,"Y coordinate of the ROI",false, roi_y);
            define("roi_radius",  1.0,"size of the ROI",false, roi_radius);
        }

        // convenience aliases, updated by update from the config data
        double roi_x,roi_y,roi_radius;

    };

    class TaskWaitForROI : public TaskInstance<TaskWaitForROIConfig,SimTasksEnv>
    {
        public:
            TaskWaitForROI(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForROI() {};

            virtual TaskIndicator iterate();

    };
    class TaskFactoryWaitForROI : public TaskDefinition<TaskWaitForROIConfig, SimTasksEnv, TaskWaitForROI>
    {
        public:
            TaskFactoryWaitForROI(TaskEnvironmentPtr env) : 
                Parent("WaitForROI","Do nothing until we reach a given destination",true,env) {}
            virtual ~TaskFactoryWaitForROI() {};
    };
}

#endif // TASK_WAIT_FOR_ROI_H
