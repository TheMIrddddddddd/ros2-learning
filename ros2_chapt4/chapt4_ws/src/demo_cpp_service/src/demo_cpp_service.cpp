#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string pkg_path = ament_index_cpp::get_package_share_directory("demo_cpp_service");
    std::string image_path = pkg_path + "/resource/zidane.jpg";

    cv::Mat image = cv::imread(image_path);
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::CascadeClassifier face_cascade;

    face_cascade.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");

    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale(gray, faces, 1.1, 15, 0, cv::Size(80, 80));


    for (auto &face : faces)
    {
        int top = face.y;
        int bottom = face.y + face.height;
        int left  =face.x;
        int right = face.x + face.width;

        cv::rectangle(image, cv::Point(left,top),cv::Point(right,bottom),cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("Result", image);
    cv::waitKey(0);

    rclcpp::shutdown();

    return 0;
}
