#include <csignal>

#include "falcon_node.hpp"

// global pointers
bool* debug_mode_pt;
Falcon* falcon_pt;

void signal_handler(int sig) {
    if (sig == SIGTSTP) {
        if (*debug_mode_pt) {
            *debug_mode_pt = false;
            falcon_pt->print_info();
        } else {
            *debug_mode_pt = true;
        }
    }
}

int main(int argc, char* argv[]) {
    // initialize falcon variables
    auto falcon = Falcon();
    auto debug_mode = false;

    // initialize global pointers
    debug_mode_pt = &debug_mode;
    falcon_pt = &falcon;

    // register custom handler to ctrl-z
    signal(SIGTSTP, signal_handler);

    // initialize ros variables
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Falcon_Node>(falcon_pt, debug_mode_pt));
    rclcpp::shutdown();

    return 0;
}
