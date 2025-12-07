#include "rclcpp/rclcpp.hpp"
#include "status_interface/msg/system_status.hpp"
#include <unistd.h>
#include <sys/sysinfo.h>
#include <fstream>
#include <sstream>
#include <string>
#include "chrono"

using namespace std::chrono_literals;

class SysStatusPub : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<status_interface::msg::SystemStatus>::SharedPtr publisher_;

    // CPU 计算需要保存上一次的值
    long prev_idle_ = 0;
    long prev_total_ = 0;

    // 获取 CPU 时间信息
    void get_cpu_times(long &idle, long &total)
    {
        std::ifstream file("/proc/stat");
        std::string cpu;
        long user, nice, system, idle_time, iowait, irq, softirq, steal;

        file >> cpu >> user >> nice >> system >> idle_time >> iowait >> irq >> softirq >> steal;

        idle = idle_time + iowait;
        total = user + nice + system + idle_time + iowait + irq + softirq + steal;
    }

    // 计算 CPU 使用率
    float get_cpu_percent()
    {
        long idle, total;
        get_cpu_times(idle, total);

        // 第一次调用，没有上次数据
        if (prev_total_ == 0)
        {
            prev_idle_ = idle;
            prev_total_ = total;
            return 0.0;
        }

        long idle_diff = idle - prev_idle_;
        long total_diff = total - prev_total_;

        prev_idle_ = idle;
        prev_total_ = total;

        if (total_diff == 0) return 0.0;

        return (1.0 - (float)idle_diff / total_diff) * 100.0;
    }

    // 获取网络流量
    void get_net_info(double &net_recv, double &net_sent)
    {
        std::ifstream file("/proc/net/dev");
        std::string line;
        net_recv = 0;
        net_sent = 0;

        while (std::getline(file, line))
        {
            // 跳过标题行和回环接口
            if (line.find("lo:") != std::string::npos ||
                line.find("|") != std::string::npos ||
                line.find("Inter") != std::string::npos)
                continue;

            if (line.find(":") != std::string::npos)
            {
                // 去掉接口名，只保留数据部分
                std::istringstream iss(line.substr(line.find(":") + 1));
                double recv, sent, tmp;

                // /proc/net/dev 格式：接收字节 包数 错误 丢弃 ... 发送字节(第9个字段)
                iss >> recv;  // 第1个：接收字节
                for (int i = 0; i < 7; i++) iss >> tmp;  // 跳过 2-8
                iss >> sent;  // 第9个：发送字节

                net_recv += recv;
                net_sent += sent;
            }
        }

        // 转换为 MB
        net_recv = net_recv / 1024.0 / 1024.0;
        net_sent = net_sent / 1024.0 / 1024.0;
    }

public:
    explicit SysStatusPub(const std::string& node_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<status_interface::msg::SystemStatus>("sys_status", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&SysStatusPub::timer_callback, this));

        RCLCPP_INFO(get_logger(), "系统状态发布节点已启动");
    }

    void timer_callback()
    {
        auto msg = status_interface::msg::SystemStatus();

        // 1. 时间戳
        msg.stamp = this->now();

        // 2. 主机名
        char hostname[256];
        gethostname(hostname, sizeof(hostname));
        msg.host_name = std::string(hostname);

        // 3. 内存信息
        struct sysinfo si;
        sysinfo(&si);
        msg.memory_total = si.totalram / 1024.0 / 1024.0;      // MB
        msg.memory_available = si.freeram / 1024.0 / 1024.0;   // MB
        msg.memory_percent = (1.0 - (float)si.freeram / si.totalram) * 100;

        // 4. CPU 使用率
        msg.cpu_percent = get_cpu_percent();

        // 5. 网络流量（累计值）
        double net_recv, net_sent;
        get_net_info(net_recv, net_sent);
        msg.net_recv = net_recv;
        msg.net_sent = net_sent;

        // 打印日志
        RCLCPP_INFO(get_logger(), "------------------------");
        RCLCPP_INFO(get_logger(), "主机: %s", msg.host_name.c_str());
        RCLCPP_INFO(get_logger(), "CPU使用率: %.1f%%", msg.cpu_percent);
        RCLCPP_INFO(get_logger(), "内存: %.1f%% (可用: %.0fMB / 总共: %.0fMB)",
            msg.memory_percent, msg.memory_available, msg.memory_total);
        RCLCPP_INFO(get_logger(), "网络: 接收 %.2fMB, 发送 %.2fMB", msg.net_recv, msg.net_sent);

        publisher_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SysStatusPub>("sys_status_pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
