#include"csignal"
#include"state_control.h"

std::shared_ptr<State_Control::StateControl> state_control;

void signalHandler(int signum)
{
        if (state_control)
        {
                state_control->Stop();
                state_control.reset();
        }
        ROS_INFO("robot_control shutting down");
        ros::shutdown();
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, State_Control::NODE_NAME);
        state_control=std::make_shared<State_Control::StateControl>();
        signal(SIGINT, signalHandler);
        state_control->Init();
        state_control->Run();

        return 0;
}
