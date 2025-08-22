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
#include <vector>

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Include our custom LCM messages (generated under package path vicon_msgs/)
#include "../vicon_msgs/vicon_msgs/vicon_state_t.hpp"
#include "../vicon_msgs/vicon_msgs/vicon_twist_t.hpp"

class LCMViconBridge {
private:
    lcm_t *lcm_;
    std::string lcm_channel_;
    std::string tracker_hostname_;
    std::string tracker_port_;
    std::string stream_mode_;
    std::string tracker_name_;
    float filter_alpha_;   // fallback smoothing factor (0..1)
    float filter_tau_s_;   // time-constant for EMA (sec). If >0, overrides alpha.


    struct PoseSample {
        int64_t utime;
        float position[3];
        float quaternion[4]; // w, x, y, z
    };
    std::unordered_map<std::string, PoseSample> last_pose_by_segment_;
    
    // Twist message for all bodies
    vicon_msgs::vicon_twist_t twist_msg_;
    int twist_body_count_;

    // ---- helpers ----
    static void normalize_quaternion(float& w, float& x, float& y, float& z) {
        double norm = std::sqrt((double)w*w + (double)x*x + (double)y*y + (double)z*z);
        if (norm > 0.0) {
            w = (float)(w / norm); x = (float)(x / norm);
            y = (float)(y / norm); z = (float)(z / norm);
        }
    }

    static float quat_dot(const float a[4], const float b[4]) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
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

    static void quat_rotate_vec(const float q[4], const float v[3], float out[3]) {
        // out = q * (0,v) * q^{-1}
        float qv[4] = {0.f, v[0], v[1], v[2]};
        float qinv[4]; quaternion_inverse(q, qinv);
        float tmp[4];  quaternion_multiply(q, qv, tmp);
        float res[4];  quaternion_multiply(tmp, qinv, res);
        out[0] = res[1]; out[1] = res[2]; out[2] = res[3];
    }

    // rotate by q^{-1} (i.e., world->body if q maps body->world)
    static void quat_rotate_vec_inv(const float q[4], const float v[3], float out[3]) {
        float qinv[4]; quaternion_inverse(q, qinv);
        quat_rotate_vec(qinv, v, out);
    }

public:
    // Velocity frame selector
    enum class VelFrame { WORLD, BODY };
    VelFrame vel_frame_ = VelFrame::BODY;  // default
    std::string twist_channel_;             // "VICON_TWIST_WORLD" or "VICON_TWIST_BODY"

    LCMViconBridge(const std::string& host, const std::string& port,
                   const std::string& channel, const std::string& mode,
                   float alpha, float tau_s, VelFrame vel_frame)
        : tracker_hostname_(host), tracker_port_(port), lcm_channel_(channel),
          stream_mode_(mode), tracker_name_("vicon_tracker"),
          filter_alpha_(alpha), filter_tau_s_(tau_s), vel_frame_(vel_frame),
          twist_body_count_(0) {

        // Initialize LCM
        lcm_ = lcm_create(NULL);
        if (!lcm_) {
            throw std::runtime_error("LCM initialization failed!");
        }
        
        // Initialize twist message
        twist_msg_.tracker_name = tracker_name_;
        twist_msg_.frame_id = (vel_frame_ == VelFrame::BODY) ? "vicon_body" : "vicon_world";
        twist_msg_.num_bodies = 0;

        twist_channel_ = (vel_frame_ == VelFrame::BODY) ? "VICON_TWIST_BODY" : "VICON_TWIST_WORLD";
    }

    ~LCMViconBridge() {
        if (lcm_) lcm_destroy(lcm_);
    }

    void run() {
        std::cout << "Starting MBot LCM Vicon Bridge..." << std::endl;
        std::cout << "Configuration:" << std::endl;
        std::cout << "  Vicon Server: " << tracker_hostname_ << ":" << tracker_port_ << std::endl;
        std::cout << "  LCM State Channel: " << lcm_channel_ << std::endl;
        std::cout << "  Twist Channel: " << twist_channel_ << std::endl;
        std::cout << "  Stream Mode: " << stream_mode_ << std::endl;
        std::cout << "  Message Type: vicon_state_t (multiple bodies)" << std::endl;
        std::cout << "  Rate: Maximum (no artificial delays)" << std::endl;
        std::cout << "  Velocity frame: " << ((vel_frame_ == VelFrame::BODY) ? "BODY" : "WORLD") << std::endl;

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
                
                // Create state message for all tracked objects
                vicon_msgs::vicon_state_t state_msg;

                // Set timestamp with latency compensation (like ROS2)
                auto now = std::chrono::high_resolution_clock::now();
                auto duration = now.time_since_epoch();
                int64_t current_time_us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                
                // Get Vicon latency and compensate (like ROS2)
                const double total_latency = sdk_client.GetLatencyTotal().Total;
                int64_t latency_us = static_cast<int64_t>(total_latency * 1000000.0); // seconds -> µs
                
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
                state_msg.tracker_name = tracker_name_;

                const unsigned int objects = sdk_client.GetSubjectCount().SubjectCount;
                int valid_bodies = 0;

                // Process each subject/object
                for (unsigned int idx_o = 0; idx_o < objects && valid_bodies < 10; idx_o++) {
                    const std::string object_name = sdk_client.GetSubjectName(idx_o).SubjectName;
                    
                    // ALL segments of the object
                    const unsigned int segments = sdk_client.GetSegmentCount(object_name).SegmentCount;
                    
                    for (unsigned int idx_s = 0; idx_s < segments && valid_bodies < 10; idx_s++) {
                        const std::string segment_name = sdk_client.GetSegmentName(object_name, idx_s).SegmentName;
                        
                        const auto segment_position =
                            sdk_client.GetSegmentGlobalTranslation(object_name, segment_name);
                        const auto segment_rotation =
                            sdk_client.GetSegmentGlobalRotationQuaternion(object_name, segment_name);

                        // Check if data is valid
                        if (segment_position.Result == ViconDataStreamSDK::CPP::Result::Success &&
                            segment_rotation.Result == ViconDataStreamSDK::CPP::Result::Success) {

                            // Occlusion (either translation or rotation)
                            bool is_occluded = segment_position.Occluded || segment_rotation.Occluded;
                            
                            // Set body name and occlusion
                            state_msg.body_names[valid_bodies] = object_name;
                            state_msg.occluded[valid_bodies] = is_occluded ? 1 : 0;

                            state_msg.positions[valid_bodies][0] = segment_position.Translation[0] * 0.001f; // x
                            state_msg.positions[valid_bodies][1] = segment_position.Translation[1] * 0.001f; // y
                            state_msg.positions[valid_bodies][2] = segment_position.Translation[2] * 0.001f; // z

                            state_msg.quaternions[valid_bodies][0] = segment_rotation.Rotation[3]; // w
                            state_msg.quaternions[valid_bodies][1] = segment_rotation.Rotation[0]; // x
                            state_msg.quaternions[valid_bodies][2] = segment_rotation.Rotation[1]; // y
                            state_msg.quaternions[valid_bodies][3] = segment_rotation.Rotation[2]; // z

                            // RPY (for convenience/inspection)
                            float qw = segment_rotation.Rotation[3];
                            float qx = segment_rotation.Rotation[0];
                            float qy = segment_rotation.Rotation[1];
                            float qz = segment_rotation.Rotation[2];
                            
                            float sinr_cosp = 2.f * (qw * qx + qy * qz);
                            float cosr_cosp = 1.f - 2.f * (qx * qx + qy * qy);
                            state_msg.rpy[valid_bodies][0] = std::atan2(sinr_cosp, cosr_cosp);
                            
                            float sinp = 2.f * (qw * qy - qz * qx);
                            if (std::fabs(sinp) >= 1.f) {
                                state_msg.rpy[valid_bodies][1] = std::copysign(M_PI / 2.f, sinp);
                            } else {
                                state_msg.rpy[valid_bodies][1] = std::asin(sinp);
                            }
                            
                            float siny_cosp = 2.f * (qw * qz + qx * qy);
                            float cosy_cosp = 1.f - 2.f * (qy * qy + qz * qz);
                            state_msg.rpy[valid_bodies][2] = std::atan2(siny_cosp, cosy_cosp);

                            // Compute and add twist for this body
                            compute_twist_for_body(object_name, segment_name, frame_time_us,
                                                   state_msg.positions[valid_bodies],
                                                   state_msg.quaternions[valid_bodies]);

                            valid_bodies++;
                        }
                    }
                }

                // Set the actual number of valid bodies we found
                state_msg.num_bodies = valid_bodies;

                // Publish state and twist
                publishState(state_msg);
                if (twist_msg_.num_bodies > 0) {
                    publishTwist(twist_msg_);
                }

                // Optional logging every 100 frames
                if (frame_count % 100 == 0) {
                    std::cout << "Published frame " << frame_count
                              << " with " << state_msg.num_bodies << " tracked bodies" << std::endl;
                }
                
            } else {
                std::cout << "Failed to get frame from Vicon. Result: " << frame_result.Result << std::endl;
            }

            // Handle LCM events (non-blocking)
            lcm_handle_timeout(lcm_, 1); // 1ms timeout to avoid blocking
        }
        
        // Not reached:
        // sdk_client.DisableSegmentData();
        // sdk_client.Disconnect();
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
        // Publish to frame-specific channel
        lcm_publish(lcm_, twist_channel_.c_str(), buffer.data(), buffer.size());
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

        // Linear velocity in world frame (finite difference)
        float v_world[3] = {
            static_cast<float>((position_m[0] - prev.position[0]) / dt),
            static_cast<float>((position_m[1] - prev.position[1]) / dt),
            static_cast<float>((position_m[2] - prev.position[2]) / dt)
        };

        // ---- Angular velocity via relative quaternion log ----
        // Enforce hemisphere continuity to avoid sign flips
        float q_prev[4] = { prev.quaternion[0], prev.quaternion[1], prev.quaternion[2], prev.quaternion[3] };
        float q_curr[4] = { quaternion_wxyz[0], quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3] };
        normalize_quaternion(q_prev[0], q_prev[1], q_prev[2], q_prev[3]);
        normalize_quaternion(q_curr[0], q_curr[1], q_curr[2], q_curr[3]);
        if (quat_dot(q_curr, q_prev) < 0.0f) {  // same rotation, opposite sign
            q_curr[0] = -q_curr[0]; q_curr[1] = -q_curr[1];
            q_curr[2] = -q_curr[2]; q_curr[3] = -q_curr[3];
        }

        // Relative rotation Δq = q_prev^{-1} ⊗ q_curr
        float q_prev_inv[4]; quaternion_inverse(q_prev, q_prev_inv);
        float dq[4];          quaternion_multiply(q_prev_inv, q_curr, dq);

        // Ensure shortest arc (dq_w >= 0)
        if (dq[0] < 0.0f) { dq[0] = -dq[0]; dq[1] = -dq[1]; dq[2] = -dq[2]; dq[3] = -dq[3]; }

        // Angle-axis from Δq
        double w = std::max(-1.0, std::min(1.0, (double)dq[0]));
        double theta = 2.0 * std::acos(w); // in [0, π]
        double s = std::sqrt(std::max(1e-16, 1.0 - w*w)); // sin(theta/2)
        double axis_d[3] = { dq[1]/s, dq[2]/s, dq[3]/s };
        if (s < 1e-8) { // very small rotation: fall back to normalized dq_xyz
            double n = std::sqrt((double)dq[1]*dq[1] + (double)dq[2]*dq[2] + (double)dq[3]*dq[3]) + 1e-12;
            axis_d[0] = dq[1]/n; axis_d[1] = dq[2]/n; axis_d[2] = dq[3]/n;
        }

        // Body-frame angular velocity (from relative rotation)
        float w_body[3] = {
            (float)((theta / dt) * axis_d[0]),
            (float)((theta / dt) * axis_d[1]),
            (float)((theta / dt) * axis_d[2])
        };

        // ---- Dirty derivative / EMA with time-constant ----
        static std::unordered_map<std::string, float> filt_wx, filt_wy, filt_wz;
        static std::unordered_map<std::string, bool>   filt_init;

        // If tau>0, compute per-sample alpha = dt / (tau + dt). Otherwise use fixed filter_alpha_
        float alpha = (filter_tau_s_ > 0.0f) ? (float)(dt / (filter_tau_s_ + dt)) : filter_alpha_;
        alpha = std::min(1.0f, std::max(0.0f, alpha));

        if (!filt_init[key]) {
            filt_wx[key] = w_body[0]; filt_wy[key] = w_body[1]; filt_wz[key] = w_body[2];
            filt_init[key] = true;
        } else {
            filt_wx[key] += alpha * (w_body[0] - filt_wx[key]);
            filt_wy[key] += alpha * (w_body[1] - filt_wy[key]);
            filt_wz[key] += alpha * (w_body[2] - filt_wz[key]);
        }

        float wx_b = filt_wx[key], wy_b = filt_wy[key], wz_b = filt_wz[key];

        // ----- Express velocities in requested frame -----
        float v_out[3];
        float w_out[3];

        if (vel_frame_ == VelFrame::BODY) {
            // Linear: v_b = R(q_prev)^T v_w
            quat_rotate_vec_inv(q_prev, v_world, v_out);
            // Angular already in body (w_body)
            w_out[0] = wx_b; w_out[1] = wy_b; w_out[2] = wz_b;
        } else {
            // WORLD frame: linear already in world
            v_out[0] = v_world[0]; v_out[1] = v_world[1]; v_out[2] = v_world[2];
            // Angular: w_w = R(q_prev) w_b
            float w_tmp[3] = {wx_b, wy_b, wz_b};
            quat_rotate_vec(q_prev, w_tmp, w_out);
        }

        // Add to twist message array
        if (twist_body_count_ < 10) {
            twist_msg_.body_names[twist_body_count_] = subject;  // Use subject name only
            twist_msg_.vx[twist_body_count_] = v_out[0];
            twist_msg_.vy[twist_body_count_] = v_out[1];
            twist_msg_.vz[twist_body_count_] = v_out[2];
            twist_msg_.wx[twist_body_count_] = w_out[0];
            twist_msg_.wy[twist_body_count_] = w_out[1];
            twist_msg_.wz[twist_body_count_] = w_out[2];
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
    float filter_alpha = 0.3f;   // fallback EMA coefficient
    float filter_tau_s  = 0.05f; // if >0, use τ-based EMA (recommended)
    std::string vel_frame_cli = "body";  // default: body (linear/body, angular/body)

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
        } else if (arg == "--alpha" && i + 1 < argc) {
            filter_alpha = std::stof(argv[++i]);
        } else if (arg == "--tau" && i + 1 < argc) {
            filter_tau_s = std::stof(argv[++i]); // e.g., 0.03 (≈5.3 Hz cutoff)
        } else if (arg == "--vel-frame" && i + 1 < argc) {
            vel_frame_cli = argv[++i]; // "world" or "body"
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --host HOST       Vicon server hostname (default: 10.10.10.5)\n"
                      << "  --port PORT       Vicon server port (default: 801)\n"
                      << "  --channel CH      LCM state channel name (default: VICON_STATE)\n"
                      << "  --mode MODE       Streaming mode: ServerPush|ClientPull|ClientPullPreFetch (default: ServerPush)\n"
                      << "  --alpha ALPHA     EMA coefficient (0..1). Used if --tau not set (default: 0.3)\n"
                      << "  --tau   TAU_S     EMA time-constant in seconds (recommended; overrides --alpha)\n"
                      << "  --vel-frame F     Velocity frame for BOTH linear and angular: world|body (default: body)\n"
                      << "                    Publishes to VICON_TWIST_WORLD or VICON_TWIST_BODY accordingly.\n"
                      << "  --help            Show this help message\n";
            return 0;
        }
    }

    // Map CLI to enum
    LCMViconBridge::VelFrame vel_frame =
        (vel_frame_cli == "body") ? LCMViconBridge::VelFrame::BODY
                                  : LCMViconBridge::VelFrame::WORLD;

    try {
        LCMViconBridge bridge(tracker_hostname, tracker_port, lcm_channel, stream_mode,
                      filter_alpha, filter_tau_s, vel_frame);
        bridge.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
