#ifndef COORDINATOR_HPP_
#define COORDINATOR_HPP_
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

// Vimba API
#include "coordinator/avt_vimba_api.hpp"
#include "coordinator/avt_vimba_camera.hpp"

#include <unordered_map>
#include <unordered_set>   
#include <deque>
#include <vector>

struct NodeStatus {
    bool alive = false;
    bool ready = false;
    rclcpp::Time last_hb;
};

class Coordinator : public rclcpp::Node, public std::enable_shared_from_this<Coordinator>
{
public:
    Coordinator();
    ~Coordinator();

	void Start();

private:
	// GigE Camera
	avt_vimba_camera::AvtVimbaApi api_;
	avt_vimba_camera::AvtVimbaCamera cam_;
	
	std::string ip_;
	std::string guid_;
	std::string camera_info_url_;
	std::string frame_id_;
	int32_t ptp_offset_;
	double acquisition_fps_;
	
	void LoadParams();
	void FrameCallback(const FramePtr& vimba_frame_ptr);
    //void SaveStatus(const std_msgs::msg::Int32::SharedPtr msg);
    //void CheckAlive(const rclcpp::Time& frame_time);
    std::vector<int> get_N_alive();
    std::vector<int> get_N_ready();
    std::vector<int> get_N_available();
    std::pair<int, int> calculate_split_grid(int N);
    void split_scheduling();

	rclcpp::TimerBase::SharedPtr alive_timer_;
	rclcpp::Time saved_time_;
	
	rclcpp::TimerBase::SharedPtr ready_wait_timer_;
	std::atomic<bool> ready_waiting_{false};
	
    // 하트비트 저장: 노드 인덱스 → 시간 로그
    std::unordered_map<int, NodeStatus> node_status_map_;

    // ROS2 인터페이스
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_subscriber_;
    std::map<int, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> hb_subscribers_;
    std::map<int, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> ready_subscribers_;

    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr> roi_publishers_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr roi_total_publisher_;

    // 파라미터 설정
    const rclcpp::Duration HB_TIMEOUT_MS = rclcpp::Duration::from_seconds(0.05);
    int width_ = 0;
    int height_ = 0;
};

#endif

