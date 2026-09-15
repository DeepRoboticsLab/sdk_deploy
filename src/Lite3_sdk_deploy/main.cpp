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

int main(int argc, char** argv){
    std::cout << "State Machine Start Running" << std::endl;
    rclcpp::init(argc, argv);
    std::shared_ptr<StateMachineBase> fsm = std::make_shared<q::QStateMachine>(RobotName::Lite3, 
        RemoteCommandType::kRos2); // Select kKeyBoard, kRetroidGamepad, or kRos2 here.
    fsm->Start();
    fsm->Run();
    fsm->Stop();

    rclcpp::shutdown();
    return 0;
}
