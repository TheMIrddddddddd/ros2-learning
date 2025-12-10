#include "rclcpp/rclcpp.hpp"

class PersonNode : public rclcpp::Node
{
private:
    std::string name_;
    int age_;

public:
    PersonNode(const std::string &node_name, const std::string &name, const int &age) : Node(node_name)
    {
        this->name_ = name;
        this->age_ = age;
    };

    void eat(const std::string &food_name)
    {
        RCLCPP_INFO(this->get_logger(), "我是%s，%d岁，爱吃%s", this->name_.c_str(), this->age_, food_name.c_str());
    };
};


int main(int argc, const char **argv)
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<PersonNode>("person_node","张三",25);

    RCLCPP_INFO(node->get_logger(), "hello world");

    node->eat("ros");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}