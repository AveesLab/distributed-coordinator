#ifndef COORDINATOR_HPP_
#define COORDINATOR_HPP_
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include "avt_vimba_camera_msgs/msg/partial_image.hpp"

// Vimba API
#include "coordinator/avt_vimba_api.hpp"
#include "coordinator/avt_vimba_camera.hpp"

#include <unordered_map>
#include <unordered_set>  
#include <deque>
#include <vector>

struct TimestampData {
    uint64_t end_ready_flag, ready_node_index, start_framecallback,start_framesensor;
    uint64_t start_update, end_update;
    uint64_t start_split_preprocess,end_split_preprocess;
    uint64_t start_split, end_split;
    uint64_t start_roi_ethernet, roi_node_index; 
    uint64_t start_roi_total_ethernet;
};

class Coordinator : public rclcpp::Node, public std::enable_shared_from_this<Coordinator>
{
public:
    Coordinator();
    ~Coordinator();

	void Start();
    uint64_t get_time_in_ms();

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
	
    rclcpp::CallbackGroup::SharedPtr callback_group_reentrant_;

	void LoadParams();
	void FrameCallback(const FramePtr& vimba_frame_ptr);
    void hb_check(std_msgs::msg::Int32::SharedPtr msg);
    void status_check(std_msgs::msg::Bool::SharedPtr msg, int node_id);
    std::pair<int, int> calculate_split_grid(int N);
    std::vector<int> hb_update();

    void split_scheduling(const std::vector<int>& ready_node);
    void SaveTimestamp(const TimestampData &data);

	rclcpp::TimerBase::SharedPtr alive_timer_;
	rclcpp::Time saved_time_;

    // ROS2 인터페이스
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_subscriber_;
    std::map<int, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> hb_subscribers_;
    std::map<int, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> status_subscribers_;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr> roi_publishers_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr roi_total_publisher_;


    //save result for csv
    std::ofstream csv_file_;
    TimestampData ts;
    std::mutex ts_mutex_;
    std::mutex status_true_mutex;
    std::mutex status_false_mutex;
    std::mutex hb_mutex;
    std::mutex map_mutex_;

    // 파라미터 설정
    static constexpr int MAX_NODES = 4;
    std::vector<int> ready_node = std::vector<int>(MAX_NODES + 1, 0);
    std::vector<int> hb_node = std::vector<int>(MAX_NODES + 1, 0);
    std::vector<int> hb_temp = std::vector<int>(MAX_NODES + 1, 0);
    std::vector<int> fail_node;
    std::unordered_map<int, rclcpp::Time> last_hb_map_;

    const rclcpp::Duration HB_TIMEOUT_NS = rclcpp::Duration::from_seconds(0.05);
    int width_ = 0;
    int height_ = 0;
    int synchronization_cnt_;
    bool cluster_flag_;
    sensor_msgs::msg::Image raw_image;
    cv::Mat load_image_;
    bool frame_active_{false};
    rclcpp::Time frame_started_at_;
    rclcpp::TimerBase::SharedPtr frame_timer_; 
    bool check = true;
    //rclcpp::Time new_frame_;
    //rclcpp::Time current_frame_ = 0;
    std::atomic<uint64_t> new_frame_ns_;
    std::atomic<uint64_t> current_frame_{0};
    std::atomic<int> reset_flag = 0;
};

#endif

