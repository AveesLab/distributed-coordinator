#include "coordinator/coordinator.hpp"

using namespace std::placeholders;

Coordinator::Coordinator() : Node("coordinator"), api_(this->get_logger()), cam_(std::shared_ptr<rclcpp::Node>(dynamic_cast<rclcpp::Node * >(this)))
{
	this->LoadParams();
	
	cam_.setCallback(std::bind(&Coordinator::FrameCallback, this, _1));
    //status_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/node_status", 50, std::bind(&Coordinator::SaveStatus, this, _1));
    //Heartbeat subscriber (1~5: computing(1~4), aggregation(5))
    for (int node_id = 1; node_id <= 5; ++node_id)
    {
        std::string hb_topic = "/hb/node_" + std::to_string(node_id);
        hb_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Bool>(
            hb_topic, 10,
            [this, node_id](std_msgs::msg::Bool::SharedPtr msg)
            {
                if (msg->data){
                    node_status_map_[node_id].alive = true;
                    node_status_map_[node_id].last_hb = this->get_clock()->now(); 
                }
            });
    }

    // Ready flag subscriber (1~4: computing node only)
    for (int node_id = 1; node_id <= 4; ++node_id)
    {
        std::string ready_topic = "/ready_flag/node_" + std::to_string(node_id);
        ready_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Bool>(
            ready_topic, 10,
            [this, node_id](std_msgs::msg::Bool::SharedPtr msg)
            {
                node_status_map_[node_id].ready = msg->data;
                auto time = this->get_clock()->now();

                std::cout << "Node" << node_id << ": ready at " << time.nanoseconds() << std::endl;
                
                if (msg->data && !ready_waiting_)
				{
				    ready_waiting_ = true;
				    // 50ms 대기 후 split scheduling 수행
				    ready_wait_timer_ = this->create_wall_timer(
				        std::chrono::milliseconds(50),
				        [this]()
				        {
				            split_scheduling();  // 가장 최신 frame 정보 사용
				            ready_waiting_ = false;
				            ready_wait_timer_->cancel();
				        }
				    );
				}
            });
    }

	for (int i = 1; i <= 4; ++i)
	{
		std::string topic = "/assigned_roi/node_" + std::to_string(i);
		roi_publishers_[i] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic, 10);
	}	
    roi_total_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/assigned_roi_total", 10);

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

std::vector<int> Coordinator::get_N_alive()
{
    std::vector<int> result;
    rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    for (const auto& [node_id, status] : node_status_map_)
    {
        //rclcpp::Time last_hb_time = rclcpp::Time(status.last_hb);
        rclcpp::Time last_hb_time(status.last_hb.nanoseconds(),RCL_SYSTEM_TIME);

        if (last_hb_time.nanoseconds() == 0) continue;

        if ((now - last_hb_time) < HB_TIMEOUT_MS)
        {
            result.push_back(node_id);
        }
    }
    return result;
}

std::vector<int> Coordinator::get_N_ready()
{
    std::vector<int> result;

    for (const auto& [node_id, status] : node_status_map_)
    {
        if (node_id <= 4 && status.ready)  // node 1~4만 체크
        {
            result.push_back(node_id);
        }
    }
    return result;
}

std::vector<int> Coordinator::get_N_available()
{
    auto n_alive = get_N_alive();
    auto n_ready = get_N_ready();

    std::unordered_set<int> ready_set(n_ready.begin(), n_ready.end());
    std::vector<int> result;

    for (int node_id : n_alive)
    {
        if (node_id <= 4 && ready_set.count(node_id))
        {
            result.push_back(node_id);
        }
    }
    return result;
}

std::pair<int, int> Coordinator::calculate_split_grid(int N)
{
    std::vector<int> divisors;
    for (int i = 1; i <= N; ++i)
    {
        if (N % i == 0)
            divisors.push_back(i);
    }

    if (divisors.size() == 2)
    {
        return {1, N};
    }
    else if (divisors.size() % 2 == 1)
    {
        int mid = divisors.size() / 2;
        return {divisors[mid], divisors[mid]};
    }
    else
    {
        int mid = divisors.size() / 2;
        return {divisors[mid - 1], divisors[mid]};
    }
}

void Coordinator::FrameCallback(const FramePtr& vimba_frame_ptr)
{
    //rclcpp::Time frame_time = msg->header.stamp;
    VmbUint64_t ts;
    vimba_frame_ptr->GetTimestamp(ts);
    rclcpp::Time frame_time = rclcpp::Time(cam_.getTimestampRealTime(ts) * 1e9);
    //RCLCPP_INFO(this->get_logger(), "Frame timestamp: %.6f", frame_time.seconds());

    saved_time_ = frame_time;

    VmbUint32_t w, h;
    vimba_frame_ptr->GetWidth(w);
    vimba_frame_ptr->GetHeight(h);
    width_  = w;
    height_ = h;

    split_scheduling();
}


void Coordinator::split_scheduling()
{
    auto avail_nodes = get_N_available();
    std::sort(avail_nodes.begin(), avail_nodes.end());

    if (avail_nodes.empty()) {
        //RCLCPP_WARN(this->get_logger(), "No available nodes for split.");
        return;
    }

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

            //ready flag 초기화
            if (node_status_map_.find(node_index) != node_status_map_.end()) 
            {
                node_status_map_[node_index].ready = false;
            }
            count++;
        }
    }
    
    for (const auto& [node_index, msg] : roi_msgs)
    {
        if (roi_publishers_.count(node_index))
            roi_publishers_[node_index]->publish(msg);

        RCLCPP_INFO(this->get_logger(), "ROI sent to node_%d: x=%d y=%d w=%d h=%d",
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]);
    }
    
    roi_total_publisher_->publish(total_roi_msg);
    //RCLCPP_INFO(this->get_logger(), "Total ROI map sent (N=%d regions)", count);

}


