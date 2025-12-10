/**
 * ============================================================================
 *                        人脸检测服务端 (Face Detect Server)
 * ============================================================================
 *
 * 【客户端与服务端联动关系】
 *
 *   客户端 (face_detect_client.cpp)                服务端 (本文件)
 *        │                                            │
 *        │  1. 创建客户端，连接服务名 "face_detect"      │  1. 创建服务，注册服务名 "face_detect"
 *        │     ↓                                      │     ↓
 *        │  2. 等待服务端上线                          │  2. spin() 持续运行，等待请求
 *        │     ↓                                      │     ↓
 *        │  3. 发送 Request (图片)  ═══════════════→  │  3. 触发回调，处理图片
 *        │     ↓                                      │     ↓
 *        │  4. 等待结果...                            │  4. 人脸检测，填充 Response
 *        │     ↓                                      │     ↓
 *        │  5. 收到 Response (人脸位置) ←═════════════  │  5. 返回 Response
 *        │     ↓                                      │
 *        │  6. 在图片上画框，显示结果                    │
 *        ↓                                            ↓
 *
 * 【服务接口定义】FaceDetector.srv
 *   Request:  sensor_msgs/Image image    → 客户端发送的图片
 *   Response: int16 number               → 服务端返回的人脸数量
 *             float32 use_time           → 服务端处理耗时
 *             int32[] top/bottom/left/right → 每张人脸的边界坐标
 *
 * 【服务端的职责】
 *   1. 注册服务，等待客户端请求
 *   2. 收到请求后，执行人脸检测
 *   3. 把检测结果填入 Response，返回给客户端
 */

#include "chapt4_interface/srv/face_detector.hpp"  // 服务接口（客户端和服务端必须使用相同的接口！）
#include "cv_bridge/cv_bridge.h"                   // OpenCV与ROS2图像消息的桥梁
#include "opencv2/opencv.hpp"                      // OpenCV图像处理
#include "rclcpp/rclcpp.hpp"                       // ROS2 C++客户端库
#include "sensor_msgs/msg/image.hpp"               // ROS2图像消息类型


/**
 * 人脸检测服务端类
 * 继承自 rclcpp::Node，成为ROS2节点
 */
class FaceDetectServer : public rclcpp::Node
{
private:
    /**
     * 服务端指针
     * - 类型：rclcpp::Service<服务接口类型>::SharedPtr
     * - 作用：注册服务，接收请求，返回响应
     * - 注意：SharedPtr 是智能指针，所以后面用 -> 访问成员
     */
    rclcpp::Service<chapt4_interface::srv::FaceDetector>::SharedPtr service_;

    /**
     * OpenCV 人脸检测器
     * - 使用 Haar 级联分类器
     * - 这是一个普通对象，所以用 . 访问成员
     */
    cv::CascadeClassifier face_cascade_;

public:
    /**
     * 构造函数
     * @param node_name 节点名称
     */
    explicit FaceDetectServer(const std::string &node_name) : Node(node_name)
    {
        // ============================================================
        // 【联动点1】创建服务，注册服务名 "face_detect"
        // ============================================================
        /**
         * create_service 需要两个参数：
         * 1. 服务名称 "face_detect" → 必须和客户端的服务名一致！
         * 2. 回调函数 → 当收到客户端请求时自动执行
         *
         * 客户端代码：create_client<...>("face_detect")
         * 服务端代码：create_service<...>("face_detect", callback)
         *                                  ↑ 必须相同！
         *
         * std::bind 用于绑定回调函数：
         * - &FaceDetectServer::detect_callback → 回调函数
         * - this → 当前对象
         * - _1, _2 → 占位符，代表 request 和 response
         */
        service_ = this->create_service<chapt4_interface::srv::FaceDetector>(
            "face_detect",
            std::bind(&FaceDetectServer::detect_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        // ============================================================
        // 【初始化】加载人脸检测模型
        // ============================================================
        /**
         * 加载 OpenCV 预训练的 Haar 级联分类器
         * 这个模型文件是 OpenCV 自带的，用于检测正脸
         */
        if (!face_cascade_.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"))
        {
            RCLCPP_ERROR(get_logger(), "无法加载人脸检测模型！");
        }

        RCLCPP_INFO(get_logger(), "人脸检测服务已启动，等待请求...");
    }

    // ============================================================
    // 【联动点2】回调函数：当客户端发送请求时自动执行
    // ============================================================
    /**
     * 人脸检测回调函数
     *
     * 【触发时机】
     * 当客户端执行 async_send_request(request) 时，这个函数会被自动调用
     *
     * 【参数说明】
     * @param request  客户端发送的请求（包含图片）
     *                 - 类型：Request::SharedPtr（智能指针，用 ->）
     *                 - 字段：request->image（客户端发送的图片）
     *
     * @param response 要返回给客户端的响应（我们需要填充它）
     *                 - 类型：Response::SharedPtr（智能指针，用 ->）
     *                 - 字段：response->number, response->top 等
     *
     * 【重要】
     * - request 是客户端发来的，只读
     * - response 是要返回的，需要我们填充
     * - 函数执行完毕后，response 会自动发送回客户端
     */
    void detect_callback(const chapt4_interface::srv::FaceDetector::Request::SharedPtr request,
                         chapt4_interface::srv::FaceDetector::Response::SharedPtr response)
    {
        // ============================================================
        // 【步骤1】记录开始时间
        // ============================================================
        auto start_time = this->now();

        // ============================================================
        // 【联动点3】从 Request 中提取图片
        // ============================================================
        /**
         * 把客户端发送的 ROS2 图像消息转换为 OpenCV 格式
         *
         * 客户端：cv::Mat → sensor_msgs::Image (发送)
         * 服务端：sensor_msgs::Image → cv::Mat (接收后转换) ← 这里
         *
         * cv_bridge::toCvCopy() 完成格式转换
         */
        cv::Mat image = cv_bridge::toCvCopy(request->image, "bgr8")->image;

        // ============================================================
        // 【步骤2】图像预处理：转灰度图
        // ============================================================
        /**
         * 人脸检测需要灰度图
         * - 减少计算量
         * - Haar 分类器要求输入灰度图
         */
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // ============================================================
        // 【步骤3】执行人脸检测（核心！）
        // ============================================================
        /**
         * detectMultiScale 参数说明：
         * - gray: 输入的灰度图
         * - faces: 输出的人脸矩形框列表
         * - 1.1: 缩放因子（每次缩小 10%）
         * - 15: 最小邻居数（过滤误检）
         * - 0: 标志位
         * - cv::Size(80,80): 最小人脸尺寸
         */
        std::vector<cv::Rect> faces;
        face_cascade_.detectMultiScale(gray, faces, 1.1, 15, 0, cv::Size(80, 80));

        // ============================================================
        // 【联动点4】填充 Response 数据
        // ============================================================
        /**
         * 把检测结果填入 response，返回给客户端
         *
         * Response 字段（由 .srv 文件定义）：
         * - number: 检测到的人脸数量
         * - top[]: 每张人脸的上边界 y 坐标
         * - bottom[]: 每张人脸的下边界 y 坐标
         * - left[]: 每张人脸的左边界 x 坐标
         * - right[]: 每张人脸的右边界 x 坐标
         *
         * 客户端会用这些坐标在图片上画矩形框
         */
        response->number = faces.size();

        for (const auto &face : faces)
        {
            // face.x, face.y 是矩形左上角坐标
            // face.width, face.height 是矩形的宽高
            response->top.push_back(face.y);                    // 上边界
            response->bottom.push_back(face.y + face.height);   // 下边界
            response->left.push_back(face.x);                   // 左边界
            response->right.push_back(face.x + face.width);     // 右边界
        }

        // ============================================================
        // 【步骤4】计算处理耗时
        // ============================================================
        auto end_time = this->now();
        response->use_time = (end_time - start_time).seconds();

        RCLCPP_INFO(get_logger(), "检测到%d张人脸,耗时%.3f秒", response->number, response->use_time);

        // ============================================================
        // 【联动点5】函数返回后，response 自动发送给客户端
        // ============================================================
        /**
         * 不需要手动发送 response！
         * 当这个回调函数执行完毕后，ROS2 会自动把 response 发送给客户端
         * 客户端的 spin_until_future_complete() 会收到结果
         */
    }
};

/**
 * 主函数
 */
int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建服务端节点
    auto node = std::make_shared<FaceDetectServer>("face_detect_server");

    // ============================================================
    // 【联动点6】spin() 持续运行，等待客户端请求
    // ============================================================
    /**
     * spin(node) 让节点持续运行，监听请求
     *
     * 【与客户端的区别】
     * - 服务端：spin(node) → 持续运行，等待多个请求
     * - 客户端：spin_until_future_complete() → 等到一个结果就停
     *
     * 【工作流程】
     * 1. spin() 开始运行，服务端进入等待状态
     * 2. 客户端发送请求 → 触发 detect_callback()
     * 3. 回调执行完毕 → response 自动返回给客户端
     * 4. 服务端继续等待下一个请求
     * 5. 重复 2-4，直到手动关闭 (Ctrl+C)
     */
    rclcpp::spin(node);

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
