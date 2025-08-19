#include <iostream>
#include <lcm/lcm.h>
#include <unistd.h>

#include "../vicon_msgs/vicon_state_t.hpp"

void handle_vicon_state(const lcm_recv_buf_t* rbuf, const char* channel, void* user) {
    vicon_msgs::vicon_state_t state;
    state.decode(rbuf->data, 0, rbuf->data_size);
    
               std::cout << "Received vicon_state_t on channel: " << channel << std::endl;
           std::cout << "  Timestamp: " << state.utime << " us" << std::endl;
           std::cout << "  Tracker: " << state.tracker_name << std::endl;
           std::cout << "  Frame ID: " << state.frame_id << std::endl;
           std::cout << "  Bodies: " << state.num_bodies << std::endl;
    std::cout << "---" << std::endl;
    
    // Print details for each tracked body
    for (int i = 0; i < state.num_bodies; i++) {
        std::cout << "  Body " << i << ": " << state.body_names[i] << std::endl;
        std::cout << "    Position: [" << state.positions[i][0] << ", " << state.positions[i][1] 
                  << ", " << state.positions[i][2] << "] m" << std::endl;
                       std::cout << "    Quaternion (wxyz): [" << state.quaternions[i][0] << ", " << state.quaternions[i][1]
                         << ", " << state.quaternions[i][2] << ", " << state.quaternions[i][3] << "]" << std::endl;
               std::cout << "    RPY: [" << state.rpy[i][0] << ", " << state.rpy[i][1]
                         << ", " << state.rpy[i][2] << "] rad" << std::endl;
               std::cout << "    Occluded: " << (state.occluded[i] ? "YES" : "NO") << std::endl;
        std::cout << "  ---" << std::endl;
    }
    std::cout << "==================" << std::endl;
}

int main(int argc, char** argv) {
    std::string channel = "VICON_STATE";
    
    if (argc > 1) {
        channel = argv[1];
    }
    
    std::cout << "Starting Vicon State Receiver..." << std::endl;
    std::cout << "Listening on channel: " << channel << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << "==================" << std::endl;
    
    lcm_t *lcm = lcm_create(NULL);
    if (!lcm) {
        std::cerr << "Error: LCM initialization failed!" << std::endl;
        return -1;
    }
    
    // Subscribe to the Vicon state channel
    lcm_subscribe(lcm, channel.c_str(), handle_vicon_state, NULL);
    
    // Main loop
    while (true) {
        lcm_handle(lcm);
        usleep(10000); // 10ms
    }
    
    lcm_destroy(lcm);
    return 0;
}
