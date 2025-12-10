# 02 C++ 人脸检测

## 1. 创建ROS2 C++包

```bash
cd ~/ros2_chapt4/chapt4_ws/src
ros2 pkg create demo_cpp_service --build-type ament_cmake --dependencies rclcpp OpenCV
```

- `--build-type ament_cmake`：指定为C++包（使用CMake构建）
- `--dependencies`：声明依赖的包

---

## 2. package.xml 详解

```xml
<?xml version="1.0"?>
<package format="3">
  <name>demo_cpp_service</name>           <!-- 包名 -->
  <version>0.0.0</version>                <!-- 版本号 -->
  <description>人脸检测示例</description>   <!-- 包描述 -->
  <maintainer email="xxx@qq.com">gan</maintainer>  <!-- 维护者信息 -->
  <license>Apache-2.0</license>           <!-- 开源协议 -->

  <!-- 构建工具依赖：使用ament_cmake构建系统 -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 运行时依赖 -->
  <depend>rclcpp</depend>           <!-- ROS2 C++客户端库 -->
  <depend>OpenCV</depend>           <!-- OpenCV计算机视觉库 -->
  <depend>ament_index_cpp</depend>  <!-- 用于获取包的安装路径 -->

  <export>
    <build_type>ament_cmake</build_type>  <!-- 声明构建类型 -->
  </export>
</package>
```

### 依赖标签说明

| 标签 | 含义 |
|------|------|
| `<buildtool_depend>` | 构建工具依赖（如ament_cmake） |
| `<build_depend>` | 仅编译时需要的依赖 |
| `<exec_depend>` | 仅运行时需要的依赖 |
| `<depend>` | 编译+运行都需要（等于build_depend + exec_depend） |

---

## 3. CMakeLists.txt 详解

```cmake
cmake_minimum_required(VERSION 3.8)  # CMake最低版本要求
project(demo_cpp_service)            # 项目名称，必须与package.xml中的name一致

# 编译器警告选项（GCC或Clang时启用）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#================================================
# 查找依赖包
#================================================
find_package(ament_cmake REQUIRED)      # ROS2构建系统（必须）
find_package(rclcpp REQUIRED)           # ROS2 C++库
find_package(OpenCV REQUIRED)           # OpenCV库
find_package(ament_index_cpp REQUIRED)  # 获取包路径的工具库

#================================================
# 创建可执行文件
#================================================
# add_executable(可执行文件名 源文件)
add_executable(demo_cpp_service src/demo_cpp_service.cpp)

#================================================
# 链接依赖（让可执行文件能使用这些库）
#================================================
ament_target_dependencies(demo_cpp_service
  rclcpp           # ROS2核心功能
  OpenCV           # 图像处理、人脸检测
  ament_index_cpp  # get_package_share_directory()函数
)

#================================================
# 安装可执行文件到install目录
#================================================
# 安装后才能用 ros2 run 运行
install(TARGETS demo_cpp_service
  DESTINATION lib/${PROJECT_NAME}
)

#================================================
# 安装资源文件（图片等）
#================================================
# 把resource/目录下的文件安装到share目录
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# 生成ament包信息（必须放最后）
ament_package()
```

### 关键函数说明

| 函数 | 作用 |
|------|------|
| `find_package()` | 查找并加载外部依赖包 |
| `add_executable()` | 创建可执行文件 |
| `ament_target_dependencies()` | 链接ROS2依赖（自动处理头文件和库） |
| `install(TARGETS ...)` | 安装可执行文件 |
| `install(DIRECTORY ...)` | 安装目录（资源文件） |
| `ament_package()` | 生成包信息，必须在最后调用 |

---

## 4. C++代码详解

```cpp
// 头文件
#include <ament_index_cpp/get_package_share_directory.hpp>  // 获取包路径
#include "rclcpp/rclcpp.hpp"      // ROS2 C++核心库
#include "opencv2/opencv.hpp"     // OpenCV主头文件（包含所有模块）

int main(int argc, char** argv)
{
    //===========================================
    // 1. 初始化ROS2
    //===========================================
    rclcpp::init(argc, argv);

    //===========================================
    // 2. 获取资源文件路径
    //===========================================
    // get_package_share_directory() 返回包的share目录路径
    // 例如: /home/gan/ros2_chapt4/chapt4_ws/install/demo_cpp_service/share/demo_cpp_service
    std::string pkg_path = ament_index_cpp::get_package_share_directory("demo_cpp_service");

    // 拼接图片完整路径（注意前面的斜杠/）
    std::string image_path = pkg_path + "/resource/zidane.jpg";

    //===========================================
    // 3. 读取图片并转灰度
    //===========================================
    // imread(): 读取图片，返回cv::Mat矩阵
    cv::Mat image = cv::imread(image_path);

    // 创建灰度图矩阵
    cv::Mat gray;

    // cvtColor(): 颜色空间转换
    // COLOR_BGR2GRAY: BGR彩色 → 灰度（人脸检测需要灰度图）
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    //===========================================
    // 4. 加载Haar级联分类器
    //===========================================
    // CascadeClassifier: 级联分类器，用于目标检测
    cv::CascadeClassifier face_cascade;

    // load(): 加载预训练的分类器模型
    // haarcascade_frontalface_default.xml: OpenCV自带的正脸检测模型
    face_cascade.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");

    //===========================================
    // 5. 执行人脸检测
    //===========================================
    // vector<Rect>: 存储检测到的人脸矩形框
    std::vector<cv::Rect> faces;

    // detectMultiScale(): 多尺度检测
    face_cascade.detectMultiScale(
        gray,             // 输入图像（灰度）
        faces,            // 输出结果（矩形框数组）
        1.1,              // scaleFactor: 图像缩放比例，每次缩小10%
        15,               // minNeighbors: 最小邻居数，越大越严格
        0,                // flags: 保留参数，一般填0
        cv::Size(80, 80)  // minSize: 最小检测尺寸
    );

    //===========================================
    // 6. 绘制检测结果
    //===========================================
    // 遍历每个检测到的人脸
    for (auto& face : faces) {
        // 将cv::Rect转换为top/right/bottom/left格式
        // 对应自定义服务 FaceDetector.srv 的响应字段
        int top = face.y;                    // 上边界 y坐标
        int bottom = face.y + face.height;   // 下边界 y坐标
        int left = face.x;                   // 左边界 x坐标
        int right = face.x + face.width;     // 右边界 x坐标

        // rectangle(): 用两个点绘制矩形
        // Point(left, top): 左上角
        // Point(right, bottom): 右下角
        // Scalar(0, 255, 0): BGR颜色，绿色
        // 2: 线宽
        cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);
    }

    //===========================================
    // 7. 显示结果
    //===========================================
    // imshow(): 创建窗口并显示图片
    cv::imshow("Result", image);

    // waitKey(0): 等待按键，0表示无限等待
    cv::waitKey(0);

    //===========================================
    // 8. 关闭ROS2
    //===========================================
    rclcpp::shutdown();

    return 0;
}
```

---

## 5. detectMultiScale 参数详解

```cpp
face_cascade.detectMultiScale(gray, faces, scaleFactor, minNeighbors, flags, minSize, maxSize);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `image` | Mat | 输入图像（灰度图） |
| `objects` | vector<Rect> | 输出的检测结果 |
| `scaleFactor` | double | 图像金字塔缩放比例，1.1表示每层缩小10% |
| `minNeighbors` | int | 候选框被保留所需的最小邻居数，**越大误检越少** |
| `flags` | int | 保留参数，一般填0 |
| `minSize` | Size | 最小检测窗口，小于此尺寸的忽略 |
| `maxSize` | Size | 最大检测窗口，大于此尺寸的忽略 |

### 调参技巧

| 问题 | 解决方案 |
|------|----------|
| 误检太多 | 调大 `minNeighbors`（5→10→15） |
| 漏检人脸 | 调小 `minNeighbors`，调小 `minSize` |
| 检测速度慢 | 调大 `scaleFactor`（1.1→1.2） |
| 小脸检测不到 | 调小 `minSize` |

---

## 6. 编译运行

```bash
# 编译
colcon build --packages-select demo_cpp_service

# 加载环境
source install/setup.bash

# 运行
ros2 run demo_cpp_service demo_cpp_service
```

---

## 7. 文件结构

```
demo_cpp_service/
├── CMakeLists.txt       # CMake构建配置
├── package.xml          # ROS2包信息和依赖声明
├── resource/
│   └── zidane.jpg       # 测试图片
└── src/
    └── demo_cpp_service.cpp  # 源代码
```

---

## 8. 常见错误

| 错误 | 原因 | 解决 |
|------|------|------|
| 图片显示空白 | imread路径错误 | 检查路径拼接是否有`/` |
| 找不到包 | 没有source | 执行`source install/setup.bash` |
| 链接错误 | 缺少依赖 | 检查CMakeLists的ament_target_dependencies |
| 分类器加载失败 | xml路径错误 | 确认`/usr/share/opencv4/haarcascades/`路径 |
