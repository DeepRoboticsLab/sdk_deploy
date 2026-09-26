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

int main(){
    std::cout << "State Machine Start Running" << std::endl;
    rclcpp::init(0, 0);
    //KeyBoard control
    std::shared_ptr<StateMachineBase> fsm = std::make_shared<qw::QwStateMachine>(RobotName::S10, RemoteCommandType::kKeyBoard);
    //Gamepad control (UDP)
    // std::shared_ptr<StateMachineBase> fsm = std::make_shared<qw::QwStateMachine>(RobotName::S10, RemoteCommandType::kGamepad);
    //DDS control (ROS 2 topic)
    // std::shared_ptr<StateMachineBase> fsm = std::make_shared<qw::QwStateMachine>(RobotName::S10, RemoteCommandType::kDDS);
    
    fsm->Start();
    fsm->Run();
    fsm->Stop();

    rclcpp::shutdown();
    return 0;
}
