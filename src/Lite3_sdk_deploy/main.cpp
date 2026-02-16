#include "quadruped/q_state_machine.hpp"

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
    std::shared_ptr<StateMachineBase> fsm = std::make_shared<q::QStateMachine>(RobotName::Lite3, 
        RemoteCommandType::kKeyBoard);
    fsm->Start();
    fsm->Run();
    fsm->Stop();

    rclcpp::shutdown();
    return 0;
}
