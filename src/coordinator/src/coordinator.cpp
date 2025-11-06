#include "coordinator/coordinator.hpp"
#include <inttypes.h>  // PRIu64 매크로 정의
#include <algorithm>   // std::count
#include <sstream>
#include <iomanip>

using namespace std::placeholders;

Coordinator::Coordinator() : Node("coordinator")
{
    this->LoadParams();

    // cam_.setCallback(std::bind(&Coordinator::FrameCallback, this, _1));

    callback_group_reentrant_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = callback_group_reentrant_;
    auto qos_feat  = rclcpp::SensorDataQoS().keep_last(64).reliable();

    // Heartbeat subscriber (1~5: computing(1~4(MAX_NODES)), aggregation(5: MAX_NODES + 1))
    for (int node_id = 1; node_id <= MAX_NODES + 1; ++node_id)
    {
        std::string hb_topic = "/hb/node_" + std::to_string(node_id);
        hb_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Int32>(
            hb_topic, 10, std::bind(&Coordinator::hb_check, this, _1), sub_opt);
    }

    for (int node_id = 1; node_id <= MAX_NODES; ++node_id)
    {
        std::string status_topic = "/task_status/node_" + std::to_string(node_id);
        status_subscribers_[node_id] = this->create_subscription<std_msgs::msg::Bool>(
            status_topic, qos_feat,
            [this, node_id](std_msgs::msg::Bool::SharedPtr msg) {
                this->status_check(msg, node_id);
            },
            sub_opt);
    }

    for (int i = 1; i <= MAX_NODES; ++i)
    {
        std::string topic = "/assigned_roi/node_" + std::to_string(i);
        roi_publishers_[i] = this->create_publisher<std_msgs::msg::Int64MultiArray>(topic, 10);
    }
    roi_total_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/assigned_roi_total", 10);

    wakeup_publisher_1 = this->create_publisher<std_msgs::msg::Bool>("/wakeup1", qos_feat);
    wakeup_publisher_2 = this->create_publisher<std_msgs::msg::Bool>("/wakeup2", qos_feat);
    wakeup_publisher_3 = this->create_publisher<std_msgs::msg::Bool>("/wakeup3", qos_feat);
    wakeup_publisher_4 = this->create_publisher<std_msgs::msg::Bool>("/wakeup4", qos_feat);

    this->synchronization_cnt_ = 0;
    this->cluster_flag_ = false;

    // CSV 파일 오픈 (요구 포맷)
    std::string filename = "coordinator_node.csv";
    csv_file_.open(filename, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
        csv_file_ << "frame_idx,"
                     "status_check_1,"
                     "status_check_2,"
                     "status_check_3,"
                     "status_check_4,"
                     "split_start_us,"
                     "publish_done_us\n";
        csv_file_.flush();
    }

    load_image_ = cv::imread("/home/avees/data-coordinator/src/coordinator/src/coordinator/test_img/output_img/0025.jpg",
                              cv::IMREAD_COLOR);
    width_  = load_image_.cols;
    height_ = load_image_.rows;
    RCLCPP_INFO(this->get_logger(), "Coordinator node initialized with standard ROS2 messages.");

    std_msgs::msg::Bool wakeup_msg;
    wakeup_msg.data = true;
    wakeup_publisher_1->publish(wakeup_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    wakeup_publisher_2->publish(wakeup_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    wakeup_publisher_3->publish(wakeup_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    wakeup_publisher_4->publish(wakeup_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    RCLCPP_INFO(this->get_logger(), "Wakeup message published: %d", wakeup_msg.data);
}

Coordinator::~Coordinator()
{
    // cam_.stop();
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
    // api_.start();
    // cam_.start(ip_, guid_, frame_id_, camera_info_url_);
    // cam_.startImaging();
}

uint64_t Coordinator::get_time_in_ms() {
    // 주의: 실제로는 us 반환(기존 호환 유지)
    return now_us();
}

void Coordinator::hb_check(std_msgs::msg::Int32::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(hb_mutex);

    if (msg->data) {
        int node_id = msg->data;
        hb_node[node_id] = 1;
        last_hb_map_[node_id] = this->now();
    }
}

std::vector<int> Coordinator::hb_update()
{
    for (int node_id = 1; node_id <= MAX_NODES; ++node_id) {
        hb_node[node_id] = 1;
    }
    // // 타임아웃 로직을 쓰려면 아래 주석 해제
    // rclcpp::Time now = this->get_clock()->now();
    // for (int node_id = 1; node_id <= MAX_NODES; ++node_id) {
    //     auto it = last_hb_map_.find(node_id);
    //     if (it != last_hb_map_.end()) {
    //         rclcpp::Duration delta = now - it->second;
    //         if (delta > HB_TIMEOUT_NS) {
    //             hb_node[node_id] = 0;
    //         }
    //     }
    // }
    return hb_node;
}

void Coordinator::status_check(std_msgs::msg::Bool::SharedPtr msg, int node_id)
{
    int node_index = node_id;

    // 노드별 마지막 status_check 콜백 시각 저장(ns)
    {
        std::lock_guard<std::mutex> lk(status_info_mutex_);
        if (node_index >= 1 && node_index <= MAX_NODES) {
            last_status_cb_time_ns_[node_index] = now_ns();
        }
    }

    if(!msg->data)
    {
        if(check)
        {
            std::lock_guard<std::mutex> lock(status_false_mutex);
            hb_temp = hb_update();
            ready_node[node_index] = 1;

            std::cout << "[ReadyNode Update] node_" << node_index << std::endl;
            std::cout << "ready_node : " << ready_node[1] << "," << ready_node[2] <<  "," << ready_node[3] << "," << ready_node[4] << std::endl;
            std::cout << "hb_temp : " << hb_temp[1] <<  "," << hb_temp[2] <<  "," << hb_temp[3] <<  "," << hb_temp[4] << std::endl;

            if (ready_node == hb_temp)
            {
                split_scheduling(ready_node);
                std::fill(ready_node.begin(), ready_node.end(), 0);
                fail_node.clear();
            }
        }
        else return;
    }
    // else { ... } // 생략
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
        return {divisors[(divisors.size() / 2) - 1], divisors[divisors.size() / 2]};
    }
    else
    {
        return {divisors[divisors.size() / 2], divisors[divisors.size() / 2]};
    }
}

/*
// 센서 프레임 콜백을 사용하려면 필요
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

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        VmbUint32_t w, h;
        vimba_frame_ptr->GetWidth(w);
        vimba_frame_ptr->GetHeight(h);
        width_  = w;
        height_ = h;
    }
}
*/

void Coordinator::split_scheduling(const std::vector<int>& ready_node)
{
    // split_scheduling 시작 시각(ns)
    const uint64_t split_start_ns = now_ns();

    rclcpp::Time now = this->now();
    int64_t start_scheduling = now.nanoseconds();

    ts.start_split_preprocess = get_time_in_ms();
    std::vector<int> avail_nodes = ready_node;
    std::vector<int> avail_nodes_index;

    if (avail_nodes.empty()) {
        return;
    }

    ts.end_split_preprocess = get_time_in_ms();

    ts.start_split = get_time_in_ms();

    int N = static_cast<int>(std::count(avail_nodes.begin(), avail_nodes.end(), 1));
    auto [cols, rows] = calculate_split_grid(N);
    std::cout << "Num of availnode : " << N << std::endl;

    for (size_t i = 0; i < avail_nodes.size(); i++) {
        if (avail_nodes[i] == 1) {
            avail_nodes_index.push_back(i);
        }
    }

    // sensor_msgs::Image -> cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat color_image;

    // Load Camera Data from workspace
    color_image = load_image_;
    const int img_w = width_;
    const int img_h = height_;
    const int crop_w = std::max(1, img_w / cols);
    const int crop_h = std::max(1, img_h / rows);

    std::vector<std::pair<int, std_msgs::msg::Int64MultiArray>> roi_msg;
    std_msgs::msg::Int32 total_roi_msg;
    int count = 0;

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int node_index = avail_nodes_index[count];
            int tile_index = ++count - 1;
            int x_offset = c * crop_w;
            int y_offset = r * crop_h;
            int w_offset = std::min(crop_w, img_w - x_offset);
            int h_offset = std::min(crop_h, img_h - y_offset);

            std_msgs::msg::Int64MultiArray assigned_roi_;
            assigned_roi_.data = {start_scheduling, tile_index, N /*size of avail nodes*/, x_offset, y_offset, w_offset, h_offset};
            roi_msg.emplace_back(node_index, assigned_roi_);
            RCLCPP_INFO(this->get_logger(),
                        "PartialImage sent to node_%d(%d of %d): x=%d y=%d w=%d h=%d",
                        node_index, tile_index, N, x_offset, y_offset, w_offset, h_offset);

            if (count == 0)
            {
                ts.roi_node_index = node_index;
            }
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
    }

    ts.start_roi_total_ethernet = get_time_in_ms();
    total_roi_msg.data = N;
    roi_total_publisher_->publish(total_roi_msg);

    // publish 완료 시각(ns)
    const uint64_t publish_done_ns = now_ns();

    // frame_idx 증가
    const uint64_t frame_idx = ++frame_idx_;

    // 노드별 마지막 status_check 시각 복사(ns)
    std::array<uint64_t, 5> snapshot_status_ns{};
    {
        std::lock_guard<std::mutex> lk(status_info_mutex_);
        snapshot_status_ns = last_status_cb_time_ns_;
    }

    // CSV 기록(초.나노초 9자리)
    WriteCsvRow(frame_idx, snapshot_status_ns, split_start_ns, publish_done_ns);
}

std::string Coordinator::format_ts_ns(uint64_t ns) const
{
    if (ns == 0) return ""; // 아직 기록 없으면 빈 칸
    uint64_t secs = ns / 1000000000ULL;
    uint32_t nsec = static_cast<uint32_t>(ns % 1000000000ULL);
    std::ostringstream oss;
    oss << secs << "." << std::setw(9) << std::setfill('0') << nsec;
    return oss.str();
}

void Coordinator::WriteCsvRow(uint64_t frame_idx,
                              const std::array<uint64_t, 5>& status_cb_ns,
                              uint64_t split_start_ns,
                              uint64_t publish_done_ns)
{
    if (!csv_file_.is_open()) return;

    csv_file_
        << frame_idx << ","
        << format_ts_ns(status_cb_ns[1]) << ","
        << format_ts_ns(status_cb_ns[2]) << ","
        << format_ts_ns(status_cb_ns[3]) << ","
        << format_ts_ns(status_cb_ns[4]) << ","
        << format_ts_ns(split_start_ns)  << ","
        << format_ts_ns(publish_done_ns) << "\n";

    csv_file_.flush();
}
