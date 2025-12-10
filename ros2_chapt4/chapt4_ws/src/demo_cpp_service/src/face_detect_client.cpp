/**
 * ============================================================================
 *                        人脸检测客户端 (Face Detect Client)
 * ============================================================================
 *
 * 【客户端与服务端联动关系】
 *
 *   客户端 (本文件)                              服务端 (face_detect_server.cpp)
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
 */

#include "chapt4_interface/srv/face_detector.hpp"  // 服务接口（客户端和服务端必须使用相同的接口！）
#include "cv_bridge/cv_bridge.h"                   // OpenCV与ROS2图像消息的桥梁
#include "opencv2/opencv.hpp"                      // OpenCV图像处理
#include "rclcpp/rclcpp.hpp"                       // ROS2 C++客户端库
#include "sensor_msgs/msg/image.hpp"               // ROS2图像消息类型
#include <ament_index_cpp/get_package_share_directory.hpp>  // 获取包路径


/**
 * 人脸检测客户端类
 * 继承自 rclcpp::Node，成为ROS2节点
 */
class FaceDetectClient : public rclcpp::Node
{
private:
    /**
     * 客户端指针
     * - 类型：rclcpp::Client<服务接口类型>::SharedPtr
     * - 作用：用于连接服务端、发送请求、接收响应
     * - 注意：SharedPtr 是智能指针，所以后面用 -> 访问成员
     */
    rclcpp::Client<chapt4_interface::srv::FaceDetector>::SharedPtr client_;


public:
    /**
     * 构造函数
     * @param node_name 节点名称
     */
    FaceDetectClient(const std::string& node_name):Node(node_name)
    {
        /**
         * 【联动点1】创建客户端，连接服务名 "face_detect"
         *
         * 这里的服务名 "face_detect" 必须和服务端的服务名一致！
         * 服务端代码：create_service<...>("face_detect", callback)
         * 客户端代码：create_client<...>("face_detect")
         *                                 ↑ 必须相同！
         */
        client_ = this->create_client<chapt4_interface::srv::FaceDetector>("face_detect");
        RCLCPP_INFO(get_logger(),"人脸检测客户端已创建");
    }

    /**
     * 发送检测请求
     * @param image_path 要检测的图片路径
     * @return 是否成功
     */
    bool send_request(const std::string &image_path)
    {
        // ============================================================
        // 【客户端五步走 - 第1步】等待服务端上线
        // ============================================================
        /**
         * 【联动点2】等待服务端启动
         *
         * wait_for_service(1秒) 会检查服务端是否在线
         * - 返回 true  → 服务端已上线，可以发送请求
         * - 返回 false → 超时，服务端还没启动
         *
         * 所以必须先启动服务端，再启动客户端！
         * 终端1: ros2 run demo_cpp_service face_detect_server  ← 先
         * 终端2: ros2 run demo_cpp_service face_detect_client  ← 后
         */
        

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_INFO(get_logger(),"等待服务时被中断");
                return false;
            }
            RCLCPP_INFO(get_logger(),"等待服务端连接...");
        }

        // ============================================================
        // 【准备数据】读取本地图片
        // ============================================================
        cv::Mat image = cv::imread(image_path);
        if(image.empty())
        {
            RCLCPP_INFO(get_logger(),"无法读取图片:%s",image_path.c_str());
            return false;
        }
        RCLCPP_INFO(get_logger(),"成功读取图片:%s",image_path.c_str());

        // ============================================================
        // 【客户端五步走 - 第2步】创建请求对象
        // ============================================================
        /**
         * 创建 Request 对象（智能指针）
         * Request 的字段由 .srv 文件定义：
         *   sensor_msgs/Image image  ← 我们要填充这个字段
         */
        auto request = std::make_shared<chapt4_interface::srv::FaceDetector::Request>();

        /**
         * 【联动点3】填充 Request 数据
         *
         * 把 OpenCV 的 cv::Mat 转换成 ROS2 的 sensor_msgs::Image
         * 服务端会用 cv_bridge::toCvCopy() 把它转回 cv::Mat
         *
         * 客户端：cv::Mat → sensor_msgs::Image (发送)
         * 服务端：sensor_msgs::Image → cv::Mat (接收后转换)
         */
        request->image = *(cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",image).toImageMsg());

        // ============================================================
        // 【客户端五步走 - 第3步】发送请求
        // ============================================================
        /**
         * 【联动点4】异步发送请求
         *
         * async_send_request(request) 做了两件事：
         * 1. 把 request 发送给服务端
         * 2. 返回一个 future 对象（用于稍后获取结果）
         *
         * 此时服务端的回调函数 detect_callback() 会被触发！
         */
        RCLCPP_INFO(get_logger(),"正在发送检测请求...");
        auto future = client_->async_send_request(request);
        
        // ============================================================
        // 【客户端五步走 - 第4步】等待服务端处理完成
        // ============================================================
        /**
         * 【联动点5】等待服务端返回结果
         *
         * spin_until_future_complete() 会阻塞等待，直到：
         * - 服务端处理完成并返回 Response → SUCCESS
         * - 超时 → TIMEOUT
         * - 被中断 → INTERRUPTED
         *
         * 在此期间，服务端正在执行：
         * 1. 把图片转成灰度图
         * 2. 执行人脸检测 detectMultiScale()
         * 3. 填充 response->number, response->top 等字段
         * 4. 返回 response
         */
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),future)!=rclcpp::FutureReturnCode::SUCCESS)
        {
           RCLCPP_ERROR(get_logger(), "服务调用失败！");
           return false;
           
        }

        // ============================================================
        // 【客户端五步走 - 第5步】获取响应
        // ============================================================
        /**
         * 【联动点6】获取服务端返回的 Response
         *
         * Response 的字段由 .srv 文件定义（服务端已填充）：
         *   int16 number      → 检测到的人脸数量
         *   float32 use_time  → 处理耗时
         *   int32[] top       → 每张人脸的上边界
         *   int32[] bottom    → 每张人脸的下边界
         *   int32[] left      → 每张人脸的左边界
         *   int32[] right     → 每张人脸的右边界
         */
        auto response = future.get();
        RCLCPP_INFO(get_logger(), "=== 检测结果 ===");
        RCLCPP_INFO(get_logger(), "检测到人脸数量: %d", response->number);
        RCLCPP_INFO(get_logger(), "处理耗时: %.3f 秒",response->use_time);

        // ============================================================
        // 【处理结果】根据服务端返回的坐标画框
        // ============================================================
        /**
         * 遍历服务端返回的每张人脸坐标，在图片上画绿色矩形框
         *
         * response->top[i], bottom[i], left[i], right[i]
         * 这些数据都是服务端检测后填充的
         */
        for(int i = 0; i < response->number;i++)
        {
            int top = response->top[i];
            int bottom = response->bottom[i];
            int left = response->left[i];
            int right = response->right[i];

            // 在图片上画绿色矩形框
            cv::rectangle(image,cv::Point(left,top),cv::Point(right,bottom),cv::Scalar(0,255,0),2);
            RCLCPP_INFO(get_logger(), "人脸 %d: top=%d, bottom=%d,left=%d, right=%d",i + 1, top, bottom, left, right);
        }

        // 显示标注后的图片
        cv::imshow("Face Detection Result", image);
        cv::waitKey(0);  // 等待按键关闭窗口

        return true;

    }

};

/**
 * 主函数
 */
int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc,argv);

    // 创建客户端节点
    auto node = std::make_shared<FaceDetectClient>("face_detect_client");

    // 获取默认图片路径
    std::string pkg_path = ament_index_cpp::get_package_share_directory("demo_cpp_service");
    std::string image_path = pkg_path + "/resource/zidane.jpg";

    // 如果命令行提供了图片路径，使用命令行参数
    if(argc > 1)
    {
        image_path = argv[1];
    }

    // 发送请求并等待结果
    node->send_request(image_path);

    // 关闭 ROS2
    rclcpp::shutdown();

    return 0;
}
