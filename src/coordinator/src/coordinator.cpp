#include "coordinator/coordinator.hpp"
#include <inttypes.h>  // PRIu64 매크로 정의

using namespace std::placeholders;

Coordinator::Coordinator() : Node("coordinator"), api_(this->get_logger()), cam_(std::shared_ptr<rclcpp::Node>(dynamic_cast<rclcpp::Node * >(this)))
{
	this->LoadParams();
	
	cam_.setCallback(std::bind(&Coordinator::FrameCallback, this, _1));
    
    callback_group_reentrant_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = callback_group_reentrant_;

    //Heartbeat subscriber (1~5: computing(1~4(MAX_NODES)), aggregation(5: MAX_NODES + 1))
    for (int node_id = 1; node_id <= MAX_NODES + 1; ++node_id)
    {
        std::string hb_topic = "/hb/node_" + std::to_string(node_id);
        hb_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Int32>(hb_topic, 10, std::bind(&Coordinator::hb_check, this, _1),sub_opt);
    }

    for (int node_id = 1; node_id <= MAX_NODES; ++node_id)
    {   
        std::string status_topic = "/task_status/node_" + std::to_string(node_id);
        status_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Bool>(status_topic, 1, 
                  [this, node_id](std_msgs::msg::Bool::SharedPtr msg) {
                    this->status_check(msg, node_id);
                    }, sub_opt);
    }

    roi_total_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/assigned_roi_total", 10);
    
    this->synchronization_cnt_ = 0;
	this->cluster_flag_ = false;

    double fps = acquisition_fps_;
    int fps_per_node = static_cast<int>(fps / static_cast<double>(MAX_NODES));
    //int fps_per_node = static_cast<int>(fps / 10);
    if (fps_per_node <= 0) fps_per_node = 1;
    double interval_sec_ = 1.0 / static_cast<double>(fps_per_node);
    RCLCPP_INFO(this->get_logger(), "Each node interval: %.3f sec", interval_sec_);

    for (int i = 1; i <= MAX_NODES; ++i)
    {
        std::string topic = "/trigger/node_" + std::to_string(i);
        time_publisher_[i] = this->create_publisher<std_msgs::msg::Int64>(topic, 10);
    }

    timer_ = this->create_wall_timer(
            std::chrono::duration<double>(interval_sec_),
            std::bind(&Coordinator::timerCallback, this));
    

    std::string filename = "coordinator_node.csv";
    csv_file_.open(filename, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
        csv_file_ << "node_id, start_timer, start_publish\n";
    }

    load_image_ = cv::imread("/home/avees/coordinator_node/src/coordinator/test_img/output_img/test_car.jpg", cv::IMREAD_COLOR);
    width_  = load_image_.cols;
    height_ = load_image_.rows;
    RCLCPP_INFO(this->get_logger(), "Coordinator node initialized with standard ROS2 messages.");
}

Coordinator::~Coordinator()
{
  cam_.stop();
}

void Coordinator::LoadParams()
{
  ip_ = this->declare_parameter("ip", "192.168.2.89");
  guid_ = this->declare_parameter("guid", "");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");
  frame_id_ = this->declare_parameter("frame_id", "coordinator");
  ptp_offset_ = this->declare_parameter("ptp_offset", 0);
  acquisition_fps_ = this->declare_parameter("AcquisitionFrameRateAbs", 30.0);
  RCLCPP_INFO(this->get_logger(), "[Initialize] Parameters loaded");
}

void Coordinator::Start()
{
  // Start Vimba & list all available cameras
  api_.start();
  // Start camera
  cam_.start(ip_, guid_, frame_id_, camera_info_url_);
  cam_.startImaging();
}

uint64_t Coordinator::get_time_in_ms() {
  rclcpp::Time now = this->get_clock()->now();
  uint64_t nanosecond = now.nanoseconds(); 
  return nanosecond/1000;
}

void Coordinator::timerCallback()
{   
    ts.start_timer = get_time_in_ms();
    rclcpp::Time now = this->get_clock()->now();
    std_msgs::msg::Int64 msg;
    msg.data = now.nanoseconds();

    // Pick next node
    int node_id = time_nodes_[current_index_];
    ts.node_id = node_id;

    // Publish
    ts.start_publish = get_time_in_ms();
    time_publisher_[node_id]->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Triggered node: %d", node_id);
    SaveTimestamp(ts);
    // Move to next node (cyclic)
    current_index_ = (current_index_ + 1) % time_nodes_.size();
}

void Coordinator::hb_check(std_msgs::msg::Int32::SharedPtr msg) 
{
    std::lock_guard<std::mutex> lock(hb_mutex);

    if (msg->data) {
        int node_id = msg->data;
        hb_node[node_id] = 1;
        last_hb_map_[node_id] = this->get_clock()->now();
    }
}

std::vector<int> Coordinator::hb_update()
{   
    rclcpp::Time now = this->get_clock()->now();
    for (int node_id = 1; node_id <= MAX_NODES; ++node_id) {
        auto it = last_hb_map_.find(node_id);
        if (it != last_hb_map_.end()) {
            rclcpp::Duration delta = now - it->second;
            if (delta > HB_TIMEOUT_NS) {
                hb_node[node_id] = 0;
            }
        }
    }
    return hb_node;   
}

void Coordinator::status_check(std_msgs::msg::Bool::SharedPtr msg, int node_id)
{
    int node_index = node_id;
    if(!msg->data)
    {
        if(check)
        {       
            std::lock_guard<std::mutex> lock(status_false_mutex);
            hb_temp = hb_update();
            ready_node[node_index] = 1;
            for (int fail_id : fail_node) 
            {
                if (fail_id >= 1 && fail_id <= MAX_NODES) 
                {
                    hb_temp[fail_id] = 0;
                }
            }

            std::cout << "[ReadyNode Update] node_" << node_index << std::endl;
            std::cout << "ready_node : " << ready_node[1] << "," << ready_node[2] <<  "," << ready_node[3] << "," << ready_node[4] << std::endl;
            std::cout << "hb_temp : " << hb_temp[1] <<  "," << hb_temp[2] <<  "," << hb_temp[3] <<  "," << hb_temp[4] << std::endl;
            if( ready_node == hb_temp )  
            {
                check = false;    
                std::fill(ready_node.begin(), ready_node.end(), 0);
                fail_node.clear();
            }
        }
        else return;
    }
    else
    {   
        check = true;
        std::lock_guard<std::mutex> lock(status_true_mutex);
        //task_status = true 온 경우
        hb_temp = hb_update();
        std::cout << "[FailNode Update] node_" << node_index << std::endl;
        if(fail_node.empty())
        {
            for(int i = 1; i <= MAX_NODES; i++)
            {
                if (hb_temp[i] == 1)
                {
                    fail_node.push_back(i);
                }
            }
        }       
        fail_node.erase(std::remove(fail_node.begin(), fail_node.end(), node_index),fail_node.end());

        std::cout << "fail_node : ";
        for (size_t i = 0; i < fail_node.size(); ++i) {
            std::cout << fail_node[i];
            if (i != fail_node.size() - 1) std::cout << ",";
        }
        std::cout << std::endl; 
    }
}

void Coordinator::FrameCallback(const FramePtr& vimba_frame_ptr)
{

    sensor_msgs::msg::Image img;
    VmbUint64_t vimba_ts;

    api_.frameToImage(vimba_frame_ptr, img);
    vimba_frame_ptr->GetTimestamp(vimba_ts);
    rclcpp::Time frame_time = rclcpp::Time(cam_.getTimestampRealTime(vimba_ts) * 1e9);
    new_frame_ns_.store(frame_time.nanoseconds(), std::memory_order_relaxed);

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        VmbUint32_t w, h;
        vimba_frame_ptr->GetWidth(w);
        vimba_frame_ptr->GetHeight(h);
        width_  = w;
        height_ = h;
    }
}

void Coordinator::SaveTimestamp(const TimestampData &data) {
    if (csv_file_.is_open()) {
        csv_file_ << data.node_id << ","
                  << data.start_timer << ","
                  << data.start_publish << ","
                  << "\n";
        csv_file_.flush();
    }
}

