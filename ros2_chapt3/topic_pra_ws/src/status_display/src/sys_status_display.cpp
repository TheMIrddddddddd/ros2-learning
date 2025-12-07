#include <QApplication>
#include <QLabel>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <status_interface/msg/system_status.hpp>

using SystemStatus = status_interface::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node
{
private:
    rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;

    QLabel *label_;

public:
    explicit SysStatusDisplay(const std::string &node_name) : Node(node_name)
    {
        // 先创建 QLabel 对象！
        label_ = new QLabel();

        subscriber_ = this->create_subscription<SystemStatus>("sys_status",10,[&]
            (const SystemStatus ::SharedPtr msg) -> void
            {
                label_->setText(get_qstr_form_msg(msg));
            }

        );

        label_->setText(get_qstr_form_msg(std::make_shared<SystemStatus>()));
        label_->show();
    };

    QString get_qstr_form_msg(const SystemStatus::SharedPtr msg)
    {
        std::stringstream show_str;

        show_str << "===============可视化显示工具===============\n"
                 << "数 据 时 间:\t" << msg->stamp.sec << "\ts\n"
                 << "主 机 名 字:\t" << msg->host_name << "\t\n"
                 << "CPU 使用率:\t" << msg->cpu_percent << "\t%\n"
                 << "内存 使用率:\t" << msg->memory_percent << "\t%\n"
                 << "内存总大小:\t" << msg->memory_total << "\tMB\n"
                 << "剩余有效内存:\t" << msg->memory_available << "\tMB\n"
                 << "网络发送量:\t" << msg->net_sent << "\tMB\n"
                 << "网络接受量:\t" << msg->net_recv << "\tMB\n"
                 << "=========================================";
        return QString::fromStdString(show_str.str());
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    QApplication a(argc, argv);

    auto node = std::make_shared<SysStatusDisplay>("sys_status_display");

    std::thread spin_thread([&]()->void{

        rclcpp::spin(node);

        }
    );

    spin_thread.detach();
    
    a.exec();

    return 0;
}