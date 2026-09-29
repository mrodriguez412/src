#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_lib/TaskServerDefault.h"
#include "floor_nav/SimTasksEnv.h"


using namespace floor_nav;
using namespace task_manager_lib;

class TaskServer : public TaskServerBase {
    protected: 
    public:
        TaskServer(TaskEnvironmentPtr _env) : TaskServerBase(_env,true) {
            start();
        }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);//init ros
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("floor_nav_tasks");
    TaskEnvironmentPtr env(new SimTasksEnv(node));
    TaskServer ts(env);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
