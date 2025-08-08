#include "coordinator/coordinator.hpp"

using namespace std::placeholders;

Coordinator::Coordinator() : Node("coordinator"), api_(this->get_logger()), cam_(std::shared_ptr<rclcpp::Node>(dynamic_cast<rclcpp::Node * >(this)))
{
	this->LoadParams();
	
	cam_.setCallback(std::bind(&Coordinator::FrameCallback, this, _1));
    
    callback_group_reentrant_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = callback_group_reentrant_;

    //Heartbeat subscriber (1~5: computing(1~4), aggregation(5))
    for (int node_id = 1; node_id <= 5; ++node_id)
    {
        std::string hb_topic = "/hb/node_" + std::to_string(node_id);
        hb_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Bool>(
            hb_topic, 10,
            [this, node_id](std_msgs::msg::Bool::SharedPtr msg)
            {
                if (msg->data){
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    last_hb_map_[node_id] = this->get_clock()->now();
                    //update_available_nodes();
                }
            }, sub_opt);
    }

    // Ready flag subscriber (1~4: computing node only)
    for (int node_id = 1; node_id <= 4; ++node_id)
    {
        std::string ready_topic = "/ready_flag/node_" + std::to_string(node_id);
        ready_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Bool>(
            ready_topic, 10,
            [this, node_id](std_msgs::msg::Bool::SharedPtr msg)
            {
                uint64_t now = get_time_in_ms();  // 현재 시간 (ms)

                {
                    std::lock_guard<std::mutex> lock(map_mutex_); 
                    ready_map_[node_id] = msg->data;

                    if (ts.end_ready_flag == 0) {
                        ts.end_ready_flag = now;
                        ts.ready_node_index = node_id;
                    }
                }

                RCLCPP_INFO(this->get_logger(),
                            "[ReadyFlag] node_%d ready received at %lu ms",
                            node_id, now);

                if (msg->data && !ready_waiting_)
                {
                    ready_waiting_ = true;
                    ready_wait_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(50),
                        [this]()
                        {
                            update_available_nodes();  // 가장 최신 frame 정보 사용
                            ready_waiting_ = false;
                            ready_wait_timer_->cancel();
                        },
                        callback_group_reentrant_
                    );
                }
            }, sub_opt);
    }

	for (int i = 1; i <= 4; ++i)
	{
		std::string topic = "/assigned_roi/node_" + std::to_string(i);
		roi_publishers_[i] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic, 10);
	}	
    roi_total_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/assigned_roi_total", 10);
    
    std::string filename = "coordinator_node.csv";
    csv_file_.open(filename, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
        csv_file_ << "framecallback_start, ready_flag_end,"
				  << "node_update_start,node_update_time(us),node_update_end,"
                  << "split_preprocess_start,split_preprocess_time(us),split_preprocess_end,"
                  << "split_start,split_time(us),split_end,"
                  << "roi_ethernet_start, roi_total_ethernet_start\n";
    }

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

void Coordinator::update_available_nodes()
{
    ts.start_update = get_time_in_ms();
    std::lock_guard<std::mutex> lock(map_mutex_);
    rclcpp::Time now = this->get_clock()->now();
    std::vector<int> result;

    for (int node_id = 1; node_id <= 4; ++node_id)
    {
        auto it_hb = last_hb_map_.find(node_id);
        auto it_ready = ready_map_.find(node_id);

        bool is_ready = (it_ready != ready_map_.end()) && it_ready->second;
        bool is_alive = false;

        if (it_hb != last_hb_map_.end())
        {
            rclcpp::Duration delta = now - it_hb->second;
            if (delta < HB_TIMEOUT_NS)
                is_alive = true;
        }

        if (is_ready && is_alive)
            result.push_back(node_id);
    }

    std::sort(result.begin(), result.end());
    cached_available_nodes_ = result;
    ts.end_update = get_time_in_ms();
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
    //rclcpp::Time frame_time = msg->header.stamp;
    ts.start_framecallback = get_time_in_ms();
    VmbUint64_t ts;
    vimba_frame_ptr->GetTimestamp(ts);
    rclcpp::Time frame_time = rclcpp::Time(cam_.getTimestampRealTime(ts) * 1e9);
    //RCLCPP_INFO(this->get_logger(), "Frame timestamp: %.6f", frame_time.seconds());
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        saved_time_ = frame_time;

        VmbUint32_t w, h;
        vimba_frame_ptr->GetWidth(w);
        vimba_frame_ptr->GetHeight(h);
        width_  = w;
        height_ = h;
    }
    
    split_scheduling();
}


void Coordinator::split_scheduling()
{
    ts.start_split_preprocess = get_time_in_ms();
    std::vector<int> avail_nodes;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);  // 🔐 읽기 보호
        avail_nodes = cached_available_nodes_;
    }

    if (avail_nodes.empty()) {
        //RCLCPP_WARN(this->get_logger(), "No available nodes for split.");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        for (int id : avail_nodes) {
            ready_map_[id] = false;
        }
    }
    ts.end_split_preprocess = get_time_in_ms();
    
    ts.start_split = get_time_in_ms();
    
    int N = static_cast<int>(avail_nodes.size());
    auto [cols,rows] = calculate_split_grid(N);

    int crop_w = width_ / cols;
    int crop_h = height_ / rows;

	// rows, cols, crop_w, crop_h 정의 이후
	std::ostringstream oss;
	oss << "Available Node: ";
	for (size_t i = 0; i < avail_nodes.size(); ++i) {
		oss << avail_nodes[i];
		if (i != avail_nodes.size() - 1) oss << ", ";
	}
	oss << " | Split grid: " << rows << " * " << cols << " (" << crop_w << " * " << crop_h << ")";
	std::cout << oss.str() << std::endl;

    int count = 0;
    std_msgs::msg::Int32MultiArray total_roi_msg;
	std::unordered_map<int, std_msgs::msg::Int32MultiArray> roi_msgs;
	
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            if (count >= N) break;

            int node_index = avail_nodes[count];
            int x_offset = c * crop_w;
            int y_offset = r * crop_h;

            std_msgs::msg::Int32MultiArray roi_msg;
            roi_msg.data = {node_index, x_offset, y_offset, crop_w, crop_h};
            
            roi_msgs[node_index] = roi_msg; 
            total_roi_msg.data.insert(total_roi_msg.data.end(), {node_index, x_offset, y_offset, crop_w, crop_h});
            
            count++;
        }
    }
    ts.end_split = get_time_in_ms();

    ts.start_roi_ethernet = get_time_in_ms();
    for (const auto& [node_index, msg] : roi_msgs)
    {
        ts.roi_node_index = node_index;
        if (roi_publishers_.count(node_index))
            roi_publishers_[node_index]->publish(msg);

        RCLCPP_INFO(this->get_logger(), "ROI sent to node_%d: x=%d y=%d w=%d h=%d",
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]);
    }
    
    ts.start_roi_total_ethernet = get_time_in_ms();
    roi_total_publisher_->publish(total_roi_msg);
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        cached_available_nodes_.clear();
    }
    //RCLCPP_INFO(this->get_logger(), "Total ROI map sent (N=%d regions)", count);
    SaveTimestamp(ts);

}

void Coordinator::SaveTimestamp(const TimestampData &data) {
    if (csv_file_.is_open()) {
        csv_file_ << data. start_framecallback << ","
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

