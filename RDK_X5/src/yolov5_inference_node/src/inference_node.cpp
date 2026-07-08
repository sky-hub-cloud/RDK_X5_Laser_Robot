#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <dnn/hb_dnn.h>
#include <dnn/hb_sys.h>

// -------------------- 模型参数 (已适配 640x640) --------------------
const int INPUT_W = 640;
const int INPUT_H = 640;
const float CONF_THRESHOLD = 0.35f;      // 根据 F1 曲线推荐，提升到 0.35 可减少误报
// YOLO26 是一对一端到端，实际上通常不需要 NMS，这里保留供兼容性
const float IOU_THRESHOLD = 0.45f;       
const float MIN_BOX_SIZE = 10.0f;
const float MAX_BOX_SIZE = 640.0f;

// 尺度对应的步长 (640/80=8, 640/40=16, 640/20=32)
const int STRIDES[3] = {8, 16, 32};

struct Detection {
    float x1, y1, x2, y2;
    float confidence;
    int class_id;   // 0 = 杂草
};

// -------------------- 工具函数：BGR 转 NV12 --------------------
cv::Mat bgr_to_nv12(const cv::Mat& bgr) {
    int h = bgr.rows, w = bgr.cols;
    CV_Assert(h % 2 == 0 && w % 2 == 0);
    cv::Mat i420;
    cv::cvtColor(bgr, i420, cv::COLOR_BGR2YUV_I420);
    cv::Mat nv12(h * 3 / 2, w, CV_8UC1);
    memcpy(nv12.data, i420.data, h * w);
    uint8_t* uv_dst = nv12.data + h * w;
    uint8_t* u_src = i420.data + h * w;
    uint8_t* v_src = i420.data + h * w + (h / 2) * (w / 2);
    for (int i = 0; i < h / 2; ++i) {
        for (int j = 0; j < w / 2; ++j) {
            uv_dst[i * w + 2 * j] = u_src[i * (w / 2) + j];
            uv_dst[i * w + 2 * j + 1] = v_src[i * (w / 2) + j];
        }
    }
    return nv12;
}

// -------------------- Letterbox 预处理 --------------------
cv::Mat letterbox(const cv::Mat& img, int target_w, int target_h,
                  float& scale, float& pad_w, float& pad_h) {
    int h = img.rows, w = img.cols;
    scale = std::min((float)target_h / h, (float)target_w / w);
    int new_w = int(w * scale);
    int new_h = int(h * scale);
    pad_w = (target_w - new_w) / 2.0f;
    pad_h = (target_h - new_h) / 2.0f;
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::Mat canvas(target_h, target_w, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect((int)pad_w, (int)pad_h, new_w, new_h)));
    return canvas;
}

// -------------------- 解码单层输出 (已适配 YOLOv10/26 Anchor-Free 公式) --------------------
std::vector<Detection> decode_output(const float* conf_data, int H, int W,
                                     const float* box_data,
                                     int stride, float conf_threshold,
                                     float min_size, float max_size) {
    std::vector<Detection> dets;
    for (int h = 0; h < H; ++h) {
        for (int w = 0; w < W; ++w) {
            int conf_idx = h * W + w;
            float obj = conf_data[conf_idx];
            obj = 1.0f / (1.0f + std::exp(-obj));   // sigmoid

            int box_idx = (h * W + w) * 4;
            float tx = box_data[box_idx];
            float ty = box_data[box_idx + 1];
            float tw = box_data[box_idx + 2];
            float th = box_data[box_idx + 3];

            // YOLOv10 / YOLO26 标准 Anchor-Free 解码公式
            tx = 1.0f / (1.0f + std::exp(-tx));
            ty = 1.0f / (1.0f + std::exp(-ty));
            tw = 1.0f / (1.0f + std::exp(-tw));
            th = 1.0f / (1.0f + std::exp(-th));

            float x_center = (tx * 2.0f - 0.5f + w) * stride;
            float y_center = (ty * 2.0f - 0.5f + h) * stride;
            float w_box = std::pow(tw * 2.0f, 2) * stride;
            float h_box = std::pow(th * 2.0f, 2) * stride;

            float x1 = x_center - w_box / 2;
            float y1 = y_center - h_box / 2;
            float x2 = x_center + w_box / 2;
            float y2 = y_center + h_box / 2;

            if (obj < conf_threshold) continue;
            if (w_box < min_size || h_box < min_size) continue;
            if (w_box > max_size || h_box > max_size) continue;
            if (x1 < 0 || y1 < 0 || x2 > INPUT_W || y2 > INPUT_H) continue;

            dets.push_back({x1, y1, x2, y2, obj, 0});
        }
    }
    return dets;
}

// -------------------- NMS (YOLO26 端到端后通常不需要, 但保留供安全冗余) --------------------
std::vector<Detection> nms(const std::vector<Detection>& dets, float iou_threshold) {
    std::vector<Detection> result;
    if (dets.empty()) return result;
    std::vector<int> order(dets.size());
    for (size_t i = 0; i < dets.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int i, int j) {
        return dets[i].confidence > dets[j].confidence;
    });
    std::vector<bool> suppressed(dets.size(), false);
    for (size_t i = 0; i < order.size(); ++i) {
        int idx = order[i];
        if (suppressed[idx]) continue;
        result.push_back(dets[idx]);
        for (size_t j = i + 1; j < order.size(); ++j) {
            int jdx = order[j];
            if (suppressed[jdx]) continue;
            float x1 = std::max(dets[idx].x1, dets[jdx].x1);
            float y1 = std::max(dets[idx].y1, dets[jdx].y1);
            float x2 = std::min(dets[idx].x2, dets[jdx].x2);
            float y2 = std::min(dets[idx].y2, dets[jdx].y2);
            float inter_area = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
            float union_area = (dets[idx].x2 - dets[idx].x1) * (dets[idx].y2 - dets[idx].y1) +
                               (dets[jdx].x2 - dets[jdx].x1) * (dets[jdx].y2 - dets[jdx].y1) - inter_area;
            float iou = inter_area / union_area;
            if (iou > iou_threshold) suppressed[jdx] = true;
        }
    }
    return result;
}

// -------------------- ROS2 节点类 --------------------
class InferenceNode : public rclcpp::Node
{
public:
    InferenceNode() : Node("inference_node_cpp")
    {
        // 声明参数 (模型路径、相机ID等均未改变)
        this->declare_parameter("model_path", "/home/sunrise/ros2_ws_10_YOLO26_weed/best_bayese_640x640_nv12.bin");
        this->declare_parameter("camera_id", 0);
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 9600);

        std::string model_path = this->get_parameter("model_path").as_string();
        camera_id_ = this->get_parameter("camera_id").as_int();
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        RCLCPP_INFO(this->get_logger(), "Loading model from %s", model_path.c_str());
        if (!init_model(model_path.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "Model initialization failed");
            rclcpp::shutdown();
            return;
        }

        // 打开USB相机
        cap_.open(camera_id_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open USB camera with id %d", camera_id_);
            rclcpp::shutdown();
            return;
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        RCLCPP_INFO(this->get_logger(), "USB camera opened successfully");

        // 发布带检测框的图像
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_with_detections", 10);

        // 初始化串口（与STM32通信）
        if (!init_serial(serial_port.c_str(), baudrate)) {
            RCLCPP_WARN(this->get_logger(), "Serial init failed, will not send data to STM32");
        }

        // 定时器：每隔一段时间处理一帧
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), // ~30fps
                                         std::bind(&InferenceNode::process_frame, this));
        RCLCPP_INFO(this->get_logger(), "Inference node initialized, starting frame processing...");
    }

    ~InferenceNode() {
        if (packed_dnn_handle_) hbDNNRelease(packed_dnn_handle_);
        if (serial_fd_ >= 0) close(serial_fd_);
        if (cap_.isOpened()) cap_.release();
    }

private:
    // 地平线推理相关
    hbPackedDNNHandle_t packed_dnn_handle_ = nullptr;
    hbDNNHandle_t dnn_handle_ = nullptr;
    hbDNNTensorProperties input_props_;
    std::vector<hbDNNTensorProperties> output_props_;

    // 相机和串口
    cv::VideoCapture cap_;
    int camera_id_;
    int serial_fd_ = -1;

    // ROS2 相关
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---------- 串口初始化 ----------
    bool init_serial(const char* port, int baudrate) {
        serial_fd_ = open(port, O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port);
            return false;
        }
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed");
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        cfsetospeed(&tty, baudrate);
        cfsetispeed(&tty, baudrate);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed");
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened (baud=%d)", port, baudrate);
        return true;
    }

// ---------- 底层发送函数（支持任意二进制长度） ----------
void send_raw_data(const uint8_t* data, size_t length) {
    if (serial_fd_ >= 0) {
        write(serial_fd_, data, length);
    }
}

// ---------- 发送检测结果（固定8字节二进制帧） ----------
void send_detection_result(const Detection& det) {
    uint8_t frame[8];

    // 1. 帧头（用于同步）
    frame[0] = 0xAA;
    frame[1] = 0x55;

    // 2. 计算坐标中心点和置信度（转为16位无符号整数）
    uint16_t x_u16 = 0;
    uint16_t y_u16 = 0;
    uint16_t conf_u16 = 0;

    if (det.confidence > 0.0f) {
        float center_x = (det.x1 + det.x2) / 2.0f;
        float center_y = (det.y1 + det.y2) / 2.0f;
        x_u16 = static_cast<uint16_t>(std::round(center_x));
        y_u16 = static_cast<uint16_t>(std::round(center_y));
        // 置信度放大1000倍，转为整数（例：0.354 -> 354）
        conf_u16 = static_cast<uint16_t>(std::round(det.confidence * 1000.0f));
    }

    // 3. 大端序打包（高字节在前，低字节在后）
    frame[2] = (x_u16 >> 8) & 0xFF;   // X 高8位
    frame[3] = x_u16 & 0xFF;          // X 低8位
    frame[4] = (y_u16 >> 8) & 0xFF;   // Y 高8位
    frame[5] = y_u16 & 0xFF;          // Y 低8位
    frame[6] = (conf_u16 >> 8) & 0xFF;// 置信度高8位
    frame[7] = conf_u16 & 0xFF;       // 置信度低8位

    // 4. 固定长度发送（8字节）
    send_raw_data(frame, 8);

    // 5. （可选）打印可读日志，方便调试
    char debug_buf[64];
    if (det.confidence > 0.0f) {
        snprintf(debug_buf, sizeof(debug_buf),
                 "D,%d,%d,%.3f,E", x_u16, y_u16, det.confidence);
    } else {
        snprintf(debug_buf, sizeof(debug_buf), "D,0,0,0,E");
    }
    RCLCPP_INFO(this->get_logger(), "Sending: %s (binary 8 bytes)", debug_buf);
}

    // ---------- 模型加载 ----------
    bool init_model(const char* model_path) {
        const char* model_files[1] = {model_path};
        int ret = hbDNNInitializeFromFiles(&packed_dnn_handle_, model_files, 1);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "hbDNNInitializeFromFiles failed: %d", ret);
            return false;
        }
        const char** model_name_list;
        int model_count = 0;
        ret = hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle_);
        if (ret != 0 || model_count == 0) {
            RCLCPP_ERROR(this->get_logger(), "hbDNNGetModelNameList failed");
            return false;
        }
        ret = hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, model_name_list[0]);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "hbDNNGetModelHandle failed");
            return false;
        }
        ret = hbDNNGetInputTensorProperties(&input_props_, dnn_handle_, 0);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "hbDNNGetInputTensorProperties failed");
            return false;
        }
        int output_count = 0;
        ret = hbDNNGetOutputCount(&output_count, dnn_handle_);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "hbDNNGetOutputCount failed");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Model has %d outputs", output_count);
        output_props_.resize(output_count);
        for (int i = 0; i < output_count; ++i) {
            ret = hbDNNGetOutputTensorProperties(&output_props_[i], dnn_handle_, i);
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "hbDNNGetOutputTensorProperties failed for output %d", i);
                return false;
            }
        }
        return true;
    }

    // 获取输出张量数据 (float)
    std::vector<float> get_output_float(const hbDNNTensor& tensor, int output_idx) {
        const hbDNNTensorProperties& props = output_props_[output_idx];
        int H = props.validShape.dimensionSize[1];
        int W = props.validShape.dimensionSize[2];
        int C = props.validShape.dimensionSize[3];
        size_t total_elements = H * W * C;
        std::vector<float> float_data(total_elements);
        memcpy(float_data.data(), tensor.sysMem[0].virAddr, total_elements * sizeof(float));
        return float_data;
    }

    // ---------- 主处理循环 ----------
    void process_frame() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        auto t_start = std::chrono::high_resolution_clock::now();

        // 预处理：Letterbox + BGR转NV12
        float scale, pad_w, pad_h;
        cv::Mat letterbox_img = letterbox(frame, INPUT_W, INPUT_H, scale, pad_w, pad_h);
        cv::Mat nv12_input = bgr_to_nv12(letterbox_img);
        int input_size = nv12_input.total() * nv12_input.elemSize();

        // 分配输入内存
        hbDNNTensor input_tensor;
        int ret = hbSysAllocCachedMem(&input_tensor.sysMem[0], input_size);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "hbSysAllocCachedMem failed for input");
            return;
        }
        memcpy(input_tensor.sysMem[0].virAddr, nv12_input.data, input_size);
        input_tensor.properties = input_props_;

        // 分配输出内存
        int output_count = output_props_.size();
        std::vector<hbDNNTensor> output_tensors(output_count);
        bool alloc_failed = false;
        int i;
        for (i = 0; i < output_count; ++i) {
            int aligned_size = output_props_[i].alignedByteSize;
            ret = hbSysAllocCachedMem(&output_tensors[i].sysMem[0], aligned_size);
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "hbSysAllocCachedMem failed for output %d", i);
                alloc_failed = true;
                break;
            }
            output_tensors[i].properties = output_props_[i];
        }
        if (alloc_failed) {
            hbSysFreeMem(&input_tensor.sysMem[0]);
            for (int j = 0; j < i; ++j)
                hbSysFreeMem(&output_tensors[j].sysMem[0]);
            return;
        }

        // 推理
        hbDNNTaskHandle_t task_handle = nullptr;
        hbDNNInferCtrlParam infer_ctrl_param;
        memset(&infer_ctrl_param, 0, sizeof(infer_ctrl_param));
        std::vector<hbDNNTensor*> output_ptrs(output_count);
        for (int i = 0; i < output_count; ++i) output_ptrs[i] = &output_tensors[i];

        ret = hbDNNInfer(&task_handle, output_ptrs.data(), &input_tensor, dnn_handle_, &infer_ctrl_param);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "hbDNNInfer failed: %d", ret);
            hbSysFreeMem(&input_tensor.sysMem[0]);
            for (int i = 0; i < output_count; ++i) hbSysFreeMem(&output_tensors[i].sysMem[0]);
            return;
        }
        ret = hbDNNWaitTaskDone(task_handle, 0);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "hbDNNWaitTaskDone failed");
        }
        for (int i = 0; i < output_count; ++i) {
            hbSysFlushMem(&output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
        }

        // 根据通道数配对输出: C=1 为置信度, C=4 为边界框
        std::vector<int> conf_indices, box_indices;
        for (int i = 0; i < output_count; ++i) {
            int C = output_props_[i].validShape.dimensionSize[3];
            if (C == 1) conf_indices.push_back(i);
            else if (C == 4) box_indices.push_back(i);
        }
        if (conf_indices.size() != 3 || box_indices.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "Mismatched output counts");
            hbSysFreeMem(&input_tensor.sysMem[0]);
            for (int i = 0; i < output_count; ++i) hbSysFreeMem(&output_tensors[i].sysMem[0]);
            if (task_handle) hbDNNReleaseTask(task_handle);
            return;
        }

        // 按高度排序（从小到大，对应 Stride 8, 16, 32）
        auto get_H = [&](int idx) { return output_props_[idx].validShape.dimensionSize[1]; };
        std::sort(conf_indices.begin(), conf_indices.end(),
                  [&](int a, int b) { return get_H(a) < get_H(b); });
        std::sort(box_indices.begin(), box_indices.end(),
                  [&](int a, int b) { return get_H(a) < get_H(b); });

        std::vector<Detection> all_dets;
        for (size_t idx = 0; idx < conf_indices.size(); ++idx) {
            int conf_idx = conf_indices[idx];
            int box_idx = box_indices[idx];
            auto conf_float = get_output_float(output_tensors[conf_idx], conf_idx);
            auto box_float = get_output_float(output_tensors[box_idx], box_idx);
            if (conf_float.empty() || box_float.empty()) continue;

            int H = output_props_[conf_idx].validShape.dimensionSize[1];
            int W = output_props_[conf_idx].validShape.dimensionSize[2];
            int stride = INPUT_H / H;
            auto dets = decode_output(conf_float.data(), H, W,
                                      box_float.data(), stride,
                                      CONF_THRESHOLD, MIN_BOX_SIZE, MAX_BOX_SIZE);
            all_dets.insert(all_dets.end(), dets.begin(), dets.end());
        }

        // YOLO26是一对一端到端模型，可不经过NMS（直接取最高置信度）
        // 如果你发现框有重叠，可以取消下一行的注释
        // auto final_dets = nms(all_dets, IOU_THRESHOLD);
        std::vector<Detection> final_dets = all_dets; 

        RCLCPP_INFO(this->get_logger(), "Detections: %zu", final_dets.size());

        // 选择置信度最高的检测结果（单目下只需一个目标用于下发）
        Detection best_det = {0,0,0,0,0,0};
        if (!final_dets.empty()) {
            best_det = *std::max_element(final_dets.begin(), final_dets.end(),
                [](const Detection& a, const Detection& b) { return a.confidence < b.confidence; });
        }

        // 发送检测结果给 STM32
        send_detection_result(best_det);

        // 绘制检测框（在原图上）
        cv::Mat display_frame = frame.clone();
        if (best_det.confidence > 0) {
            // 将模型坐标映射回原始图像尺寸
            float x1 = (best_det.x1 - pad_w) / scale;
            float y1 = (best_det.y1 - pad_h) / scale;
            float x2 = (best_det.x2 - pad_w) / scale;
            float y2 = (best_det.y2 - pad_h) / scale;
            x1 = std::max(0.0f, std::min(x1, (float)frame.cols));
            y1 = std::max(0.0f, std::min(y1, (float)frame.rows));
            x2 = std::max(0.0f, std::min(x2, (float)frame.cols));
            y2 = std::max(0.0f, std::min(y2, (float)frame.rows));

            cv::rectangle(display_frame, cv::Point((int)x1, (int)y1),
                          cv::Point((int)x2, (int)y2), cv::Scalar(0, 255, 0), 2);
            cv::putText(display_frame, "Weed",
                        cv::Point((int)x1, (int)y1 - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        // 发布图像
        auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", display_frame).toImageMsg();
        out_msg->header.stamp = this->now();
        publisher_->publish(*out_msg);

        // 释放内存
        hbSysFreeMem(&input_tensor.sysMem[0]);
        for (int i = 0; i < output_count; ++i) hbSysFreeMem(&output_tensors[i].sysMem[0]);
        if (task_handle) hbDNNReleaseTask(task_handle);

        auto t_end = std::chrono::high_resolution_clock::now();
        float fps = 1.0f / std::chrono::duration<float>(t_end - t_start).count();
        RCLCPP_INFO(this->get_logger(), "FPS: %.2f", fps);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InferenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
