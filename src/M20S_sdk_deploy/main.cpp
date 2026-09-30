#include "quadruped_wheel/qw_state_machine.hpp"

#ifdef USE_SIMULATION
    #define BACKWARD_HAS_DW 1
    #include "backward.hpp"
    namespace backward{
        backward::SignalHandling sh;
    }
#endif

using namespace types;
MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main(int argc, char** argv){
    std::cout << "State Machine Start Running" << std::endl;
    rclcpp::init(argc, argv);
    // Choose the input interface by changing RemoteCommandType below:
    // kKeyBoard = 0: keyboard (default); kGamepad = 1: gamepad;
    // kRos2 = 2: ROS 2 topics. Rebuild and restart rl_deploy after changing it.
    std::shared_ptr<StateMachineBase> fsm = std::make_shared<qw::QwStateMachine>(RobotName::M20S, RemoteCommandType::kKeyBoard);
    
    fsm->Start();
    fsm->Run();
    fsm->Stop();

    rclcpp::shutdown();
    return 0;
}
