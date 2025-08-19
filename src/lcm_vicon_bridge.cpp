#include <iostream>
#include <string>
#include <chrono>
#include <lcm/lcm.h>
#include <Client.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <unordered_map>

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Include our custom LCM messages
#include "../vicon_msgs/vicon_state_t.hpp"
#include "../vicon_msgs/vicon_twist_t.hpp"

class LCMViconBridge {
private:
    lcm_t *lcm_;
    std::string lcm_channel_;
    std::string tracker_hostname_;
    std::string tracker_port_;
    std::string stream_mode_;
    std::string tracker_name_;

    struct PoseSample {
        int64_t utime;
        float position[3];
        float quaternion[4]; // w, x, y, z
    };
    std::unordered_map<std::string, PoseSample> last_pose_by_segment_;
    
    // Twist message for all bodies
    vicon_msgs::vicon_twist_t twist_msg_;
    int twist_body_count_;

public:
    LCMViconBridge(const std::string& host, const std::string& port,
                   const std::string& channel, const std::string& mode)
        : tracker_hostname_(host), tracker_port_(port), lcm_channel_(channel),
          stream_mode_(mode), tracker_name_("vicon_tracker"), twist_body_count_(0) {

        // Initialize LCM
        lcm_ = lcm_create(NULL);
        if (!lcm_) {
            throw std::runtime_error("LCM initialization failed!");
        }
        
        // Initialize twist message
        twist_msg_.tracker_name = tracker_name_;
        twist_msg_.frame_id = "vicon_world";
        twist_msg_.num_bodies = 0;
    }

    ~LCMViconBridge() {
        if (lcm_) lcm_destroy(lcm_);
    }

    void run() {
        std::cout << "Starting MBot LCM Vicon Bridge..." << std::endl;
        std::cout << "Configuration:" << std::endl;
        std::cout << "  Vicon Server: " << tracker_hostname_ << ":" << tracker_port_ << std::endl;
        std::cout << "  LCM Channel: " << lcm_channel_ << std::endl;
        std::cout << "  Stream Mode: " << stream_mode_ << std::endl;
        std::cout << "  Message Type: vicon_state_t (multiple bodies)" << std::endl;
        std::cout << "  Rate: Maximum (no artificial delays)" << std::endl;

        // Assemble the full hostname
        std::string full_hostname = tracker_hostname_ + ":" + tracker_port_;
        std::cout << "Connecting to Vicon Tracker (DataStream SDK) at " << full_hostname << std::endl;

        // Initialize the DataStream SDK
        ViconDataStreamSDK::CPP::Client sdk_client;
        std::cout << "Connecting to server...";

        sdk_client.Connect(full_hostname);
        usleep(10000);

        while (!sdk_client.IsConnected().Connected) {
            std::cout << "...taking a while to connect, trying again..." << std::endl;
            sdk_client.Connect(full_hostname);
            usleep(10000);
        }
        std::cout << "...connected!" << std::endl;

        // Enable data
        sdk_client.EnableSegmentData();

        // Set the axes (right-handed, X-forwards, Y-left, Z-up, same as ROS)
        sdk_client.SetAxisMapping(ViconDataStreamSDK::CPP::Direction::Forward,
                                  ViconDataStreamSDK::CPP::Direction::Left,
                                  ViconDataStreamSDK::CPP::Direction::Up);
        std::cout << "Coordinate system set to: X-forward, Y-left, Z-up (same as ROS)" << std::endl;

        // Set streaming mode
        if (stream_mode_ == "ServerPush") {
            sdk_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
        } else if (stream_mode_ == "ClientPullPreFetch") {
            sdk_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch);
        } else if (stream_mode_ == "ClientPull") {
            sdk_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
        } else {
            std::cerr << "Invalid stream mode " << stream_mode_ << std::endl;
            std::cerr << "Valid options are ServerPush, ClientPullPreFetch, and ClientPull" << std::endl;
            exit(-1);
        }

        std::cout << "Streaming data..." << std::endl;

        int frame_count = 0;
        while (true) {
            auto frame_result = sdk_client.GetFrame();
            if (frame_result.Result == ViconDataStreamSDK::CPP::Result::Success) {
                frame_count++;
                std::cout << "Frame " << frame_count << " received successfully!" << std::endl;
                
                // Create state message for all tracked objects
                vicon_msgs::vicon_state_t state_msg;

                // Set timestamp with latency compensation (like ROS2)
                auto now = std::chrono::high_resolution_clock::now();
                auto duration = now.time_since_epoch();
                int64_t current_time_us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                
                // Get Vicon latency and compensate (like ROS2)
                const double total_latency = sdk_client.GetLatencyTotal().Total;
                int64_t latency_us = static_cast<int64_t>(total_latency * 1000000.0); // Convert seconds to microseconds
                
                // Apply latency compensation
                int64_t frame_time_us = current_time_us - latency_us;
                if (frame_time_us < 0) {
                    std::cout << "    WARNING: Latency compensation resulted in negative time, using current time" << std::endl;
                    frame_time_us = current_time_us;
                }
                
                // Reset twist message for this frame
                twist_msg_.utime = frame_time_us;
                twist_msg_.num_bodies = 0;
                twist_body_count_ = 0;
                
                state_msg.utime = frame_time_us;
                
                // Debug: Show timing info (like ROS2)
                std::cout << "    Timing: current=" << current_time_us << " us, latency=" << latency_us << " us, frame=" << frame_time_us << " us" << std::endl;

                // Set tracker name
                state_msg.tracker_name = tracker_name_;

                const unsigned int objects = sdk_client.GetSubjectCount().SubjectCount;
                int valid_bodies = 0;

                std::cout << "Found " << objects << " subjects" << std::endl;

                // Process each subject/object
                for (unsigned int idx_o = 0; idx_o < objects && valid_bodies < 10; idx_o++) {
                    const std::string object_name = sdk_client.GetSubjectName(idx_o).SubjectName;
                    std::cout << "Processing subject: " << object_name << std::endl;
                    
                    // Loop through ALL segments of the object (like ROS2)
                    const unsigned int segments = sdk_client.GetSegmentCount(object_name).SegmentCount;
                    std::cout << "  Found " << segments << " segments" << std::endl;
                    
                    // Process each segment
                    for (unsigned int idx_s = 0; idx_s < segments && valid_bodies < 10; idx_s++) {
                        const std::string segment_name = sdk_client.GetSegmentName(object_name, idx_s).SegmentName;
                        std::cout << "  Processing segment: " << segment_name << std::endl;
                        
                        const ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation segment_position =
                            sdk_client.GetSegmentGlobalTranslation(object_name, segment_name);
                        const ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationQuaternion segment_rotation =
                            sdk_client.GetSegmentGlobalRotationQuaternion(object_name, segment_name);

                        std::cout << "    Position result: " << segment_position.Result << std::endl;
                        std::cout << "    Rotation result: " << segment_rotation.Result << std::endl;
                        std::cout << "    Raw position (mm): [" 
                                  << segment_position.Translation[0] << ", " 
                                  << segment_position.Translation[1] << ", " 
                                  << segment_position.Translation[2] << "]" << std::endl;

                        // Check if data is valid (like ROS2) - using proper Vicon SDK enums
                        if (segment_position.Result == ViconDataStreamSDK::CPP::Result::Success &&
                            segment_rotation.Result == ViconDataStreamSDK::CPP::Result::Success) {

                            // Check occlusion like ROS2: BOTH position AND rotation
                            bool is_occluded = segment_position.Occluded || segment_rotation.Occluded;
                            std::cout << "    Occluded: " << (is_occluded ? "YES" : "NO") << std::endl;
                            
                            // Set body name
                            state_msg.body_names[valid_bodies] = object_name;
                            
                            // Store occlusion status (like ROS2)
                            state_msg.occluded[valid_bodies] = is_occluded ? 1 : 0;

                            state_msg.positions[valid_bodies][0] = segment_position.Translation[0] * 0.001; // x
                            state_msg.positions[valid_bodies][1] = segment_position.Translation[1] * 0.001; // y
                            state_msg.positions[valid_bodies][2] = segment_position.Translation[2] * 0.001; // z

                            state_msg.quaternions[valid_bodies][0] = segment_rotation.Rotation[3]; // w
                            state_msg.quaternions[valid_bodies][1] = segment_rotation.Rotation[0]; // x
                            state_msg.quaternions[valid_bodies][2] = segment_rotation.Rotation[1]; // y
                            state_msg.quaternions[valid_bodies][3] = segment_rotation.Rotation[2]; // z

                            // Calculate RPY from quaternions (instead of hardcoding to 0)
                            float qw = segment_rotation.Rotation[3];
                            float qx = segment_rotation.Rotation[0];
                            float qy = segment_rotation.Rotation[1];
                            float qz = segment_rotation.Rotation[2];
                            
                            // Roll (x-axis rotation)
                            float sinr_cosp = 2 * (qw * qx + qy * qz);
                            float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
                            state_msg.rpy[valid_bodies][0] = atan2(sinr_cosp, cosr_cosp);
                            
                            // Pitch (y-axis rotation)
                            float sinp = 2 * (qw * qy - qz * qx);
                            if (fabs(sinp) >= 1) {
                                state_msg.rpy[valid_bodies][1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
                            } else {
                                state_msg.rpy[valid_bodies][1] = asin(sinp);
                            }
                            
                            // Yaw (z-axis rotation)
                            float siny_cosp = 2 * (qw * qz + qx * qy);
                            float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
                            state_msg.rpy[valid_bodies][2] = atan2(siny_cosp, cosy_cosp);

                            std::cout << "    Valid data: pos=[" 
                                      << state_msg.positions[valid_bodies][0] << ", " 
                                      << state_msg.positions[valid_bodies][1] << ", " 
                                      << state_msg.positions[valid_bodies][2] << "] m" << std::endl;
                            std::cout << "    RPY: [" 
                                      << state_msg.rpy[valid_bodies][0] * 180.0 / M_PI << "°, " 
                                      << state_msg.rpy[valid_bodies][1] * 180.0 / M_PI << "°, " 
                                      << state_msg.rpy[valid_bodies][2] * 180.0 / M_PI << "°]" << std::endl;

                            // Compute and add twist for this body
                            compute_twist_for_body(object_name, segment_name, frame_time_us,
                                                   state_msg.positions[valid_bodies],
                                                   state_msg.quaternions[valid_bodies]);

                            valid_bodies++;
                        } else {
                            std::cout << "    No valid data for this segment" << std::endl;
                        }
                    }
                }

                // Set the actual number of valid bodies we found
                state_msg.num_bodies = valid_bodies;

                // Publish the complete state message
                std::cout << "Publishing message with " << state_msg.num_bodies << " bodies" << std::endl;
                publishState(state_msg);
                
                // Publish the twist message
                if (twist_msg_.num_bodies > 0) {
                    std::cout << "Publishing twist message with " << twist_msg_.num_bodies << " bodies" << std::endl;
                    publishTwist(twist_msg_);
                }
                
                std::cout << "Messages published successfully!" << std::endl;

                // Print status every 100 frames to avoid spam
                if (frame_count % 100 == 0) {
                    std::cout << "Published frame " << frame_count << " with " << state_msg.num_bodies << " tracked bodies" << std::endl;
                }

                // Measure actual frame rate (not artificially capped)
                static auto last_time = std::chrono::high_resolution_clock::now();
                static int rate_counter = 0;
                rate_counter++;
                
            } else {
                std::cout << "Failed to get frame from Vicon. Result: " << frame_result.Result << std::endl;
            }

            // Handle LCM events (non-blocking)
            lcm_handle_timeout(lcm_, 1); // 1ms timeout to avoid blocking
        }
        
        sdk_client.DisableSegmentData();
        sdk_client.Disconnect();
    }

    void publishState(const vicon_msgs::vicon_state_t& state_msg) {
        // Encode and publish
        const int encoded_size = state_msg.getEncodedSize();
        std::vector<uint8_t> buffer(encoded_size);
        state_msg.encode(buffer.data(), 0, buffer.size());
        lcm_publish(lcm_, lcm_channel_.c_str(), buffer.data(), buffer.size());
    }

    void publishTwist(const vicon_msgs::vicon_twist_t& twist_msg) {
        // Encode and publish twist message
        const int encoded_size = twist_msg.getEncodedSize();
        std::vector<uint8_t> buffer(encoded_size);
        twist_msg.encode(buffer.data(), 0, buffer.size());
        lcm_publish(lcm_, "VICON_TWIST", buffer.data(), buffer.size());
    }

    static std::string sanitize_for_channel(const std::string& input) {
        std::string out;
        out.reserve(input.size());
        for (char c : input) {
            char u = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            if ((u >= 'A' && u <= 'Z') || (u >= '0' && u <= '9') || u == '_') {
                out.push_back(u);
            } else {
                out.push_back('_');
            }
        }
        return out;
    }

    static void normalize_quaternion(float& w, float& x, float& y, float& z) {
        double norm = std::sqrt(static_cast<double>(w)*w + static_cast<double>(x)*x + static_cast<double>(y)*y + static_cast<double>(z)*z);
        if (norm > 0.0) {
            w = static_cast<float>(w / norm);
            x = static_cast<float>(x / norm);
            y = static_cast<float>(y / norm);
            z = static_cast<float>(z / norm);
        }
    }

    static void quaternion_inverse(const float q[4], float q_inv[4]) {
        // unit quaternion inverse is conjugate
        q_inv[0] = q[0];
        q_inv[1] = -q[1];
        q_inv[2] = -q[2];
        q_inv[3] = -q[3];
    }

    static void quaternion_multiply(const float a[4], const float b[4], float out[4]) {
        // (w, x, y, z)
        out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
        out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
        out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
        out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    }

    void compute_twist_for_body(const std::string& subject,
                                const std::string& segment,
                                int64_t utime,
                                const float position_m[3],
                                const float quaternion_wxyz[4]) {
        // Build key per subject/segment
        std::string key = subject + "/" + segment;
        auto it = last_pose_by_segment_.find(key);
        if (it == last_pose_by_segment_.end()) {
            PoseSample sample;
            sample.utime = utime;
            sample.position[0] = position_m[0];
            sample.position[1] = position_m[1];
            sample.position[2] = position_m[2];
            sample.quaternion[0] = quaternion_wxyz[0];
            sample.quaternion[1] = quaternion_wxyz[1];
            sample.quaternion[2] = quaternion_wxyz[2];
            sample.quaternion[3] = quaternion_wxyz[3];
            last_pose_by_segment_[key] = sample;
            return; // need two samples for a velocity estimate
        }

        const PoseSample& prev = it->second;
        const double dt = (utime - prev.utime) / 1e6; // seconds
        if (dt <= 0.0) {
            // Update stored sample and return
            PoseSample sample;
            sample.utime = utime;
            sample.position[0] = position_m[0];
            sample.position[1] = position_m[1];
            sample.position[2] = position_m[2];
            sample.quaternion[0] = quaternion_wxyz[0];
            sample.quaternion[1] = quaternion_wxyz[1];
            sample.quaternion[2] = quaternion_wxyz[2];
            sample.quaternion[3] = quaternion_wxyz[3];
            last_pose_by_segment_[key] = sample;
            return;
        }

        // Linear velocity in world frame
        float vx = static_cast<float>((position_m[0] - prev.position[0]) / dt);
        float vy = static_cast<float>((position_m[1] - prev.position[1]) / dt);
        float vz = static_cast<float>((position_m[2] - prev.position[2]) / dt);

        // Angular velocity in world frame via quaternion delta
        float q_prev[4] = { prev.quaternion[0], prev.quaternion[1], prev.quaternion[2], prev.quaternion[3] };
        float q_curr[4] = { quaternion_wxyz[0], quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3] };
        normalize_quaternion(q_prev[0], q_prev[1], q_prev[2], q_prev[3]);
        normalize_quaternion(q_curr[0], q_curr[1], q_curr[2], q_curr[3]);

        float q_prev_inv[4];
        quaternion_inverse(q_prev, q_prev_inv);
        float dq[4];
        quaternion_multiply(q_prev_inv, q_curr, dq); // rotation from prev to curr
        normalize_quaternion(dq[0], dq[1], dq[2], dq[3]);

        // Convert dq to axis-angle
        double angle = 2.0 * std::atan2(std::sqrt(static_cast<double>(dq[1])*dq[1] + static_cast<double>(dq[2])*dq[2] + static_cast<double>(dq[3])*dq[3]), std::max(-1.0f, std::min(1.0f, dq[0])));
        double s = std::sqrt(static_cast<double>(dq[1])*dq[1] + static_cast<double>(dq[2])*dq[2] + static_cast<double>(dq[3])*dq[3]);
        double ax = 0.0, ay = 0.0, az = 0.0;
        if (s > 1e-9) {
            ax = dq[1] / s;
            ay = dq[2] / s;
            az = dq[3] / s;
        }
        float wx = static_cast<float>((angle / dt) * ax);
        float wy = static_cast<float>((angle / dt) * ay);
        float wz = static_cast<float>((angle / dt) * az);

        // Add to twist message array
        if (twist_body_count_ < 10) {
            twist_msg_.body_names[twist_body_count_] = subject;  // Use subject name only
            twist_msg_.vx[twist_body_count_] = vx;
            twist_msg_.vy[twist_body_count_] = vy;
            twist_msg_.vz[twist_body_count_] = vz;
            twist_msg_.wx[twist_body_count_] = wx;
            twist_msg_.wy[twist_body_count_] = wy;
            twist_msg_.wz[twist_body_count_] = wz;
            twist_body_count_++;
            twist_msg_.num_bodies = twist_body_count_;
        }

        // Update stored sample
        PoseSample sample;
        sample.utime = utime;
        sample.position[0] = position_m[0];
        sample.position[1] = position_m[1];
        sample.position[2] = position_m[2];
        sample.quaternion[0] = quaternion_wxyz[0];
        sample.quaternion[1] = quaternion_wxyz[1];
        sample.quaternion[2] = quaternion_wxyz[2];
        sample.quaternion[3] = quaternion_wxyz[3];
        last_pose_by_segment_[key] = sample;
    }
};

int main(int argc, char** argv) {
    std::string tracker_hostname = "10.10.10.5";
    std::string tracker_port = "801";
    std::string lcm_channel = "VICON_STATE";
    std::string stream_mode = "ServerPush";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--host" && i + 1 < argc) {
            tracker_hostname = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            tracker_port = argv[++i];
        } else if (arg == "--channel" && i + 1 < argc) {
            lcm_channel = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            stream_mode = argv[++i];
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --host HOST     Vicon server hostname (default: 10.10.10.5)" << std::endl;
            std::cout << "  --port PORT     Vicon server port (default: 801)" << std::endl;
            std::cout << "  --channel CH    LCM channel name (default: VICON_STATE)" << std::endl;
            std::cout << "  --mode MODE     Streaming mode: ServerPush/ClientPull/ClientPullPreFetch (default: ServerPush)" << std::endl;
            std::cout << "  --help          Show this help message" << std::endl;
            return 0;
        }
    }

    try {
        LCMViconBridge bridge(tracker_hostname, tracker_port, lcm_channel, stream_mode);
        bridge.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
