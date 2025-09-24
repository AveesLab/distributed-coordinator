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

    // Ready flag subscriber (1~4 (MAX_NODES): computing node only)
    for (int node_id = 1; node_id <= MAX_NODES; ++node_id)
    {   
        std::string ready_topic = "/ready_flag/node_" + std::to_string(node_id);
        ready_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Header>(ready_topic, 10, std::bind(&Coordinator::ready_check, this, _1),sub_opt);
    }

	for (int i = 1; i <= MAX_NODES; ++i)
    {
        std::string topic = "/assigned_roi/node_" + std::to_string(i);
        roi_publishers_[i] = this->create_publisher<std_msgs::msg::Int64MultiArray>(topic, 10);
    }
    roi_total_publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/assigned_roi_total", 10);
    
    this->synchronization_cnt_ = 0;
	this->cluster_flag_ = false;

    std::string filename = "coordinator_node.csv";
    csv_file_.open(filename, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
        csv_file_ << "framesensor_start,framecallback_start, ready_flag_end,"
				  << "node_update_start,node_update_time(us),node_update_end,"
                  << "split_preprocess_start,split_preprocess_time(us),split_preprocess_end,"
                  << "split_start,split_time(us),split_end,"
                  << "roi_ethernet_start, roi_total_ethernet_start\n";
    }

    load_image_ = cv::imread("/home/avees/coordinator_node/src/coordinator/test_img/output_img/test_car.jpg", cv::IMREAD_COLOR);
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

  // Start camera`
  cam_.start(ip_, guid_, frame_id_, camera_info_url_);
  cam_.startImaging();
}

uint64_t Coordinator::get_time_in_ms() {
  rclcpp::Time now = this->get_clock()->now();
  uint64_t nanosecond = now.nanoseconds(); 
  return nanosecond/1000;
}


std::vector<int> Coordinator::vector_AND(const std::vector<int>& a, const std::vector<int>& b) {
    size_t n = std::min(a.size(), b.size());
    std::vector<int> result(n, 0);
    for (size_t i = 0; i < n; ++i) {
        result[i] = a[i] * b[i];              
    }
    std::cout << "vector AND : " << result[1] <<  "," << result[2] <<  "," << result[3] <<  "," << result[4] << std::endl;
    return result;
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

void Coordinator::ready_check(std_msgs::msg::Header::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(ready_mutex);

    //rclcpp::Time ready_frame_(msg->stamp.sec, msg->stamp.nanosec, RCL_ROS_TIME);
    uint64_t ready_frame_ = (uint64_t)msg->stamp.sec * 1000000000ULL + msg->stamp.nanosec;
    uint64_t new_frame_ = new_frame_ns_.load(std::memory_order_relaxed);

    int node_index = std::stoi(msg->frame_id);
    //RCLCPP_INFO(this->get_logger(), "ReadyFlag from node_%d, frame_id=%s, ready_frame=%lu new_frame=%lu current_frame=%" PRIu64 " ",node_index, msg->frame_id.c_str(), ready_frame_, new_frame_, current_frame_);    
    RCLCPP_INFO(this->get_logger(),
    "ReadyFlag from node_%d, frame_id=%s, ready_frame=%lu new_frame=%lu current_frame=%" PRIu64,
    node_index,
    msg->frame_id.c_str(),
    ready_frame_,
    new_frame_,
    current_frame_.load());
    if (ready_frame_ != new_frame_)
    {
        return; 
    }
    else //(ready_frame_ == new_frame_)
    {
        
        // if(current_frame_ == 0 || current_frame_ == new_frame_)
        // {                            
                ready_node[node_index] = 1;
                std::cout << "ready_node : " << ready_node[1] << "," << ready_node[2] <<  "," << ready_node[3] << "," << ready_node[4] << std::endl;
                current_frame_.store(new_frame_, std::memory_order_relaxed);
                hb_temp = hb_update();
                std::cout << "hb_temp : " << hb_temp[1] <<  "," << hb_temp[2] <<  "," << hb_temp[3] <<  "," << hb_temp[4] << std::endl;

                if( ready_node == hb_temp )  
                {
                    split_scheduling(ready_node, current_frame_);
                    std::fill(ready_node.begin(), ready_node.end(), 0);
                    std::fill(pre_ready_node.begin(), pre_ready_node.end(), 0);
                    current_frame_.store(0, std::memory_order_relaxed);
                }
                else pre_ready_node[node_index] = 1;  
        // }                        
        // else //(current_frame_ != 0 && current_frame_ != new_frame_)
        // {
        //         std::cout << "pre_ready_node : " << pre_ready_node[1] << "," << pre_ready_node[2] <<  "," << pre_ready_node[3] << "," << pre_ready_node[4] << std::endl;
        //         std::cout << "ready_node : " << ready_node[1] << "," << ready_node[2] <<  "," << ready_node[3] << "," << ready_node[4] << std::endl;
        //         std::cout << "hb_temp : " << hb_temp[1] <<  "," << hb_temp[2] <<  "," << hb_temp[3] <<  "," << hb_temp[4] << std::endl;

        //     if(reset_flag == 0) 
        //     { 
        //         std::fill(ready_node.begin(), ready_node.end(), 0); 
        //         reset_flag = 1; 
        //     }
        //     ready_node[node_index] = 1;
        //     if(ready_node == vector_AND(pre_ready_node, hb_update())) 
        //     {
                
        //         reset_flag = 0;
        //         split_scheduling(ready_node, current_frame_);
        //         std::fill(ready_node.begin(), ready_node.end(), 0);
        //         std::fill(pre_ready_node.begin(), pre_ready_node.end(), 0);
        //         current_frame_.store(0, std::memory_order_relaxed);
        //     }
        // }
    }
}

std::pair<int, int> Coordinator::calculate_split_grid(int N)
{
    std::vector<int> divisors;
    for (int i = 1; i <= N; ++i)
    {
        if (N % i == 0)
            divisors.push_back(i);
    }
    if (divisors.size() % 2 == 0)
    {
        return {divisors[(divisors.size() / 2) - 1],divisors[divisors.size() / 2]};
    }
    else
    {
    	return {divisors[divisors.size() / 2], divisors[divisors.size() / 2]};
    }
}

void Coordinator::FrameCallback(const FramePtr& vimba_frame_ptr)
{
    ts.start_framecallback = get_time_in_ms();

    sensor_msgs::msg::Image img;
    VmbUint64_t vimba_ts;

    api_.frameToImage(vimba_frame_ptr, img);
    vimba_frame_ptr->GetTimestamp(vimba_ts);
    rclcpp::Time frame_time = rclcpp::Time(cam_.getTimestampRealTime(vimba_ts) * 1e9);
    new_frame_ns_.store(frame_time.nanoseconds(), std::memory_order_relaxed);
    
    ts.start_framesensor = frame_time.nanoseconds() / 1000;

    //RCLCPP_INFO(this->get_logger(), "Frame timestamp: %.6f", frame_time.seconds());
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        saved_time_ = frame_time;

        VmbUint32_t w, h;
        vimba_frame_ptr->GetWidth(w);
        vimba_frame_ptr->GetHeight(h);
        width_  = w;
        height_ = h;

        img.header.stamp    = saved_time_;
        img.header.frame_id = "camera";
        raw_image = img; 
        
        //RCLCPP_INFO(this->get_logger(), "Image info: w=%d h=%d step=%d encoding=%s",raw_image.width, raw_image.height, raw_image.step, raw_image.encoding.c_str());
    }
}

void Coordinator::split_scheduling(const std::vector<int>& ready_node, uint64_t frame_time)
{
    ts.start_split_preprocess = get_time_in_ms();
    std::vector<int> avail_nodes = ready_node;
    std::vector<int> avail_nodes_index;  

    //int64_t frame_us = frame_time;

    if (avail_nodes.empty()) {
        //RCLCPP_WARN(this->get_logger(), "No available nodes for split.");
        return;
    }

    ts.end_split_preprocess = get_time_in_ms();
    
    ts.start_split = get_time_in_ms();

    int N = static_cast<int>(std::count(avail_nodes.begin(), avail_nodes.end(), 1));
    auto [cols,rows] = calculate_split_grid(N);
    std::cout << "Num of availnode : " << N << std::endl;

    for (size_t i = 0; i < avail_nodes.size(); i++) {
        if (avail_nodes[i] == 1) {
            avail_nodes_index.push_back(i);
        }
    }

    // sensor_msgs::Image -> cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat color_image;

    //Load Camera Data from Sensor
    //cv_ptr = cv_bridge::toCvCopy(full_img, sensor_msgs::image_encodings::MONO8);
    //cv::cvtColor(cv_ptr->image, color_image, cv::COLOR_BayerRG2BGR);

    //Load Camera Data from workspace
    color_image = load_image_;

    const int img_w = width_;
    const int img_h = height_;
    const int crop_w = std::max(1, img_w / cols);
    const int crop_h = std::max(1, img_h / rows);

    std::vector<std::pair<int, std_msgs::msg::Int64MultiArray>> roi_msg;
    roi_msg.reserve(N);
    std_msgs::msg::Int64MultiArray total_roi_msg;
    
    int count = 0;
	//RCLCPP_INFO(this->get_logger(), "Start Image Split");

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {

            int node_index = avail_nodes_index[count];
            int x_offset = c * crop_w;
            int y_offset = r * crop_h;
            int w_offset = std::min(crop_w, img_w - x_offset);
            int h_offset = std::min(crop_h, img_h - y_offset);

            std_msgs::msg::Int64MultiArray roi_;
            roi_.data = {static_cast<int64_t>(frame_time), x_offset, y_offset, w_offset, h_offset};
            roi_msg.emplace_back(node_index, roi_);
            total_roi_msg.data.insert(total_roi_msg.data.end(), {static_cast<int64_t>(frame_time), x_offset, y_offset, w_offset, h_offset});

            if (count == 0) 
            {
                ts.roi_node_index = node_index;
            }
            //RCLCPP_INFO(this->get_logger(), "PartialImage sent to node_%d: x=%d y=%d w=%d h=%d", node_index, x_offset, y_offset, w_offset, h_offset);
            count++;
        }
    }
    
    ts.end_split = get_time_in_ms();
    
    ts.start_roi_ethernet = get_time_in_ms();
    for (const auto& ROI : roi_msg) {
        int node_index = ROI.first;
        auto it = roi_publishers_.find(node_index);
        if (it != roi_publishers_.end()) 
        {
            it->second->publish(ROI.second);
        }
        //RCLCPP_INFO(this->get_logger(), "ROI sent to node_%d", node_index);
    }

    ts.start_roi_total_ethernet = get_time_in_ms();
    roi_total_publisher_->publish(total_roi_msg);

    //RCLCPP_INFO(this->get_logger(), "Total ROI map sent (N=%d regions)", count);
    SaveTimestamp(ts);
}

void Coordinator::SaveTimestamp(const TimestampData &data) {
    if (csv_file_.is_open()) {
        csv_file_ << data. start_framecallback << ","
                  << data. start_framesensor << ","
                  << data.end_ready_flag << ","
				  << data.start_update << ","
                  << (data.end_update - data.start_update) << ","
                  << data.end_update << ","
                  << data.start_split_preprocess << ","
                  << (data.end_split_preprocess - data.start_split_preprocess) << ","
                  << data.end_split_preprocess << ","
                  << data.start_split << ","
                  << (data.end_split - data.start_split) << ","
                  << data.end_split << ","
                  << data.start_roi_ethernet << ","
                  << data.start_roi_total_ethernet << ","
                  << "\n";
        csv_file_.flush();
    }
}

