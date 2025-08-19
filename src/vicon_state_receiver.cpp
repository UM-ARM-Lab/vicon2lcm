#include <iostream>
#include <lcm/lcm.h>
#include "../vicon_msgs/vicon_state_t.hpp"
#include "../vicon_msgs/vicon_twist_t.hpp"

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void handle_vicon_state(const lcm_recv_buf_t* rbuf, const char* channel, void* user) {
    vicon_msgs::vicon_state_t msg;
    int result = msg.decode(rbuf->data, 0, rbuf->data_size);
    
    if (result < 0) {
        std::cout << "Error decoding vicon state message" << std::endl;
        return;
    }
    
    std::cout << "\n=== VICON STATE ===" << std::endl;
    std::cout << "Channel: " << channel << std::endl;
    std::cout << "Timestamp: " << msg.utime << " us" << std::endl;
    std::cout << "Tracker: " << msg.tracker_name << std::endl;
    std::cout << "Frame ID: " << msg.frame_id << std::endl;
    std::cout << "Number of bodies: " << msg.num_bodies << std::endl;
    
    for (int i = 0; i < msg.num_bodies; i++) {
        std::cout << "  Body " << i << ": " << msg.body_names[i] << std::endl;
        std::cout << "    Position: [" << msg.positions[i][0] << ", " << msg.positions[i][1] << ", " << msg.positions[i][2] << "] m" << std::endl;
        std::cout << "    Quaternion: [" << msg.quaternions[i][0] << ", " << msg.quaternions[i][1] << ", " << msg.quaternions[i][2] << ", " << msg.quaternions[i][3] << "]" << std::endl;
        std::cout << "    RPY: [" << msg.rpy[i][0] * 180.0 / M_PI << "°, " << msg.rpy[i][1] * 180.0 / M_PI << "°, " << msg.rpy[i][2] * 180.0 / M_PI << "°]" << std::endl;
        std::cout << "    Occluded: " << (msg.occluded[i] ? "YES" : "NO") << std::endl;
    }
}

void handle_vicon_twist(const lcm_recv_buf_t* rbuf, const char* channel, void* user) {
    vicon_msgs::vicon_twist_t msg;
    int result = msg.decode(rbuf->data, 0, rbuf->data_size);
    
    if (result < 0) {
        std::cout << "Error decoding vicon twist message" << std::endl;
        return;
    }
    
    std::cout << "\n=== VICON TWIST ===" << std::endl;
    std::cout << "Channel: " << channel << std::endl;
    std::cout << "Timestamp: " << msg.utime << " us" << std::endl;
    std::cout << "Tracker: " << msg.tracker_name << std::endl;
    std::cout << "Frame ID: " << msg.frame_id << std::endl;
    std::cout << "Number of bodies: " << msg.num_bodies << std::endl;
    
    for (int i = 0; i < msg.num_bodies; i++) {
        std::cout << "  Body " << i << ": " << msg.body_names[i] << std::endl;
        std::cout << "    Linear velocity: [" << msg.vx[i] << ", " << msg.vy[i] << ", " << msg.vz[i] << "] m/s" << std::endl;
        std::cout << "    Angular velocity: [" << msg.wx[i] << ", " << msg.wy[i] << ", " << msg.wz[i] << "] rad/s" << std::endl;
    }
}

int main() {
    lcm_t* lcm = lcm_create(NULL);
    if (!lcm) {
        std::cerr << "Failed to create LCM instance" << std::endl;
        return -1;
    }
    
    std::cout << "Starting Vicon State & Twist Receiver..." << std::endl;
    std::cout << "Listening on channels: VICON_STATE, VICON_TWIST" << std::endl;
    
    lcm_subscribe(lcm, "VICON_STATE", handle_vicon_state, NULL);
    lcm_subscribe(lcm, "VICON_TWIST", handle_vicon_twist, NULL);
    
    while (true) {
        lcm_handle(lcm);
    }
    
    lcm_destroy(lcm);
    return 0;
}
