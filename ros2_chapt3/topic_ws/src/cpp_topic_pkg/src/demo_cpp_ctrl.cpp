#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "chrono"
#include "turtlesim/msg/pose.hpp"


using namespace std::chrono_literals;

class TurtleCtrlNode:public rclcpp::Node
{
private:
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;

    double target_x_{1.0};
    double target_y_{1.0};
    double k_{1.0};
    double max_speed_{3.0};

public:
    explicit TurtleCtrlNode(const std::string& node_name):Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&TurtleCtrlNode::subscriber_callback,this,std::placeholders::_1));



        
    };

    void subscriber_callback(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto current_x = pose->x;
        auto current_y  =pose->y;
        RCLCPP_INFO(get_logger(),"X=%3.f,Y=%3.f",current_x,current_y);

        auto distance = std::sqrt(
            (target_x_-current_x)*(target_x_-current_x)+
            (target_y_-current_y)*(target_y_-current_y)

        );
        auto angle = std::atan2((target_y_-current_y),(target_x_-current_x))-pose->theta;

        auto msg = geometry_msgs::msg::Twist();
        if(distance>0.1){
            if(fabs(angle)>0.2){
                msg.angular.z = fabs(angle);
            }else
            {
                msg.linear.x = k_* distance;
            }

        }

        if(msg.linear.x > max_speed_)
        {
            msg.linear.x = max_speed_;
        }

        publisher_ -> publish(msg);
    };

};
int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    
    auto node = std::make_shared<TurtleCtrlNode>("turtle_ctrl");
    rclcpp::spin(node);
    rclcpp::shutdown();

    
    return 0;
}





