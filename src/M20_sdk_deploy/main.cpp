#include "quadruped_wheel/qw_state_machine.hpp"
#include <cstring>

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

    RemoteCommandType rct = RemoteCommandType::kKeyBoard;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--twist") == 0) {
            rct = RemoteCommandType::kTwist;
        } else if (std::strcmp(argv[i], "--gamepad") == 0) {
            rct = RemoteCommandType::kGamepad;
        }
    }

    std::shared_ptr<StateMachineBase> fsm = std::make_shared<qw::QwStateMachine>(RobotName::M20, rct);
    
    fsm->Start();
    fsm->Run();
    fsm->Stop();

    rclcpp::shutdown();
    return 0;
}
