#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <igris_sdk/channel_factory.hpp>
#include <igris_sdk/publisher.hpp>
#include <igris_sdk/subscriber.hpp>
#include <igris_sdk/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace igris_sdk;

static constexpr std::array<uint16_t, 12> DEFAULT_HAND_MOTOR_IDS = {
    11, 12, 13, 14, 15, 16,
    21, 22, 23, 24, 25, 26,
};

static std::atomic<bool> g_running(true);
static std::mutex g_mtx;
static std::array<float, 12> g_targets{};
static std::atomic<uint32_t> g_handstate_count(0);
static std::atomic<uint32_t> g_handcmd_count(0);
static std::atomic<uint32_t> g_init_response_count(0);

void signalHandler(int) {
    g_running = false;
}

static float clamp01(float value) {
    return std::clamp(value, 0.0f, 1.0f);
}

static std::vector<uint16_t> parseIdList(const char *text) {
    std::vector<uint16_t> ids;
    if (text == nullptr || text[0] == '\0') {
        for (uint16_t id = 0; id <= 40; ++id) {
            ids.push_back(id);
        }
        return ids;
    }

    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) {
            continue;
        }
        ids.push_back(static_cast<uint16_t>(std::stoi(item)));
    }
    return ids;
}

void handStateCallback(const HandState &) {
    g_handstate_count++;
}

void handInitResponseCallback(const ServiceResponse &res) {
    g_init_response_count++;
    if (std::getenv("HAND_DEBUG_LOG") != nullptr || !res.success()) {
        std::cout << "[HandInit] response " << (res.success() ? "OK" : "FAIL")
                  << " request_id=" << res.request_id()
                  << " message=" << res.message() << std::endl;
    }
}

int main(int argc, char **argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("quest_handcmd_bridge");

    int domain_id = 0;
    if (argc >= 2) {
        domain_id = std::atoi(argv[1]);
    }

    auto target_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/igris_c/hand/targets", 10,
        [](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() < 12) {
                return;
            }
            std::lock_guard<std::mutex> lock(g_mtx);
            for (size_t i = 0; i < g_targets.size(); ++i) {
                g_targets[i] = clamp01(msg->data[i]);
            }
        });

    ChannelFactory::Instance()->Init(domain_id);
    if (!ChannelFactory::Instance()->IsInitialized()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize DDS ChannelFactory");
        rclcpp::shutdown();
        return 1;
    }

    Subscriber<HandState> handstate_sub("rt/handstate");
    if (!handstate_sub.init(handStateCallback)) {
        RCLCPP_WARN(node->get_logger(), "Failed to initialize HandState subscriber");
    }

    Subscriber<ServiceResponse> hand_init_res_sub("rt/service/hand_init/response");
    if (!hand_init_res_sub.init(handInitResponseCallback)) {
        RCLCPP_WARN(node->get_logger(), "Failed to initialize HandInit response subscriber");
    }


    Publisher<HandCmd> cmd_writer("rt/handcmd");
    if (!cmd_writer.init()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize HandCmd publisher");
        rclcpp::shutdown();
        return 1;
    }

    auto active_motor_ids = DEFAULT_HAND_MOTOR_IDS;
    const auto env_motor_ids = parseIdList(std::getenv("HAND_MOTOR_IDS"));
    if (std::getenv("HAND_MOTOR_IDS") != nullptr) {
        if (env_motor_ids.size() != active_motor_ids.size()) {
            RCLCPP_ERROR(node->get_logger(), "HAND_MOTOR_IDS must contain exactly 12 comma-separated ids");
            rclcpp::shutdown();
            return 1;
        }
        std::copy(env_motor_ids.begin(), env_motor_ids.end(), active_motor_ids.begin());
    }

    std::ostringstream id_stream;
    for (size_t i = 0; i < active_motor_ids.size(); ++i) {
        if (i > 0) {
            id_stream << ",";
        }
        id_stream << active_motor_ids[i];
    }

    const bool debug_log = std::getenv("HAND_DEBUG_LOG") != nullptr;
    RCLCPP_INFO(node->get_logger(), "SDK domain=%d", domain_id);
    RCLCPP_INFO(node->get_logger(), "Sub: /igris_c/hand/targets");
    RCLCPP_INFO(node->get_logger(), "Sub: rt/handstate, rt/service/hand_init/response");
    RCLCPP_INFO(node->get_logger(), "Pub: rt/handcmd (DDS HandCmd ids: %s)", id_stream.str().c_str());
    RCLCPP_INFO(node->get_logger(), "Periodic hand debug log: %s", debug_log ? "enabled" : "disabled");

    const bool probe_mode = std::getenv("HAND_ID_PROBE") != nullptr;
    if (probe_mode) {
        const auto probe_ids = parseIdList(std::getenv("HAND_ID_PROBE_IDS"));
        float probe_q = 0.35f;
        if (const char *env_probe_q = std::getenv("HAND_ID_PROBE_Q")) {
            probe_q = clamp01(std::stof(env_probe_q));
        }
        RCLCPP_WARN(node->get_logger(), "HAND_ID_PROBE enabled. Sweeping one HandCmd id at a time.");
        RCLCPP_WARN(node->get_logger(), "Use HAND_ID_PROBE_IDS=comma,separated,ids to limit the sweep.");
        RCLCPP_WARN(node->get_logger(), "Probe q=%.2f. Override with HAND_ID_PROBE_Q=0.0..1.0", probe_q);

        std::thread ros_thread([&]() {
            while (g_running && rclcpp::ok()) {
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });

        uint32_t count = 0;
        while (g_running && rclcpp::ok()) {
            for (uint16_t id : probe_ids) {
                if (!g_running || !rclcpp::ok()) {
                    break;
                }

                for (float q : {0.0f, probe_q, 0.0f}) {
                    auto start = std::chrono::steady_clock::now();
                    while (g_running && rclcpp::ok() &&
                           std::chrono::steady_clock::now() - start < std::chrono::milliseconds(700)) {
                        HandCmd cmd;
                        cmd.motor_cmd().resize(1);
                        auto &mc = cmd.motor_cmd()[0];
                        mc.id(id);
                        mc.q(q);
                        mc.dq(0.0f);
                        mc.tau(0.0f);
                        mc.kp(0.0f);
                        mc.kd(0.0f);
                        if (cmd_writer.write(cmd)) {
                            g_handcmd_count++;
                        }

                        if (count++ % 60 == 0) {
                            std::cout << "[HandIdProbe] id=" << id
                                      << " q=" << std::fixed << std::setprecision(2) << q
                                      << " cmds=" << g_handcmd_count.load()
                                      << " handstate=" << g_handstate_count.load()
                                      << std::endl;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(8));
                    }
                }
            }
        }

        g_running = false;
        ros_thread.join();
        rclcpp::shutdown();
        return 0;
    }


    std::thread ros_thread([&]() {
        while (g_running && rclcpp::ok()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    auto period = std::chrono::microseconds(8333);
    auto next_time = std::chrono::steady_clock::now();
    int count = 0;

    while (g_running && rclcpp::ok()) {
        std::array<float, 12> targets{};
        {
            std::lock_guard<std::mutex> lock(g_mtx);
            targets = g_targets;
        }

        HandCmd cmd;
        cmd.motor_cmd().resize(12);
        for (size_t i = 0; i < targets.size(); ++i) {
            auto &mc = cmd.motor_cmd()[i];
            mc.id(active_motor_ids[i]);
            mc.q(targets[i]);
            mc.dq(0.0f);
            mc.tau(0.0f);
            mc.kp(0.0f);
            mc.kd(0.0f);
        }
        if (cmd_writer.write(cmd)) {
            g_handcmd_count++;
        }

        ++count;
        if (debug_log && count % 120 == 0) {
            std::cout << "[HandCmd] cmds=" << g_handcmd_count.load()
                      << " handstate=" << g_handstate_count.load()
                      << " init_res=" << g_init_response_count.load()
                      << " R "
                      << std::fixed << std::setprecision(2)
                      << targets[0] << " " << targets[1] << " " << targets[2] << " "
                      << targets[3] << " " << targets[4] << " " << targets[5]
                      << " | L "
                      << targets[6] << " " << targets[7] << " " << targets[8] << " "
                      << targets[9] << " " << targets[10] << " " << targets[11]
                      << std::endl;
        }

        next_time += period;
        std::this_thread::sleep_until(next_time);
    }

    g_running = false;
    ros_thread.join();
    rclcpp::shutdown();
    return 0;
}
