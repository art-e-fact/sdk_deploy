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

    RemoteCommandType input_type = RemoteCommandType::kKeyBoard;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--gamepad") {
            input_type = RemoteCommandType::kGamepad;
            std::cout << "Using gamepad input mode" << std::endl;
        } else if (std::string(argv[i]) == "--twist") {
            input_type = RemoteCommandType::kTwist;
            std::cout << "Using twist (/cmd_vel) input mode" << std::endl;
        }
    }

    std::shared_ptr<StateMachineBase> fsm = std::make_shared<qw::QwStateMachine>(RobotName::M20, input_type);
    
    fsm->Start();
    fsm->Run();
    fsm->Stop();

    rclcpp::shutdown();
    return 0;
}
