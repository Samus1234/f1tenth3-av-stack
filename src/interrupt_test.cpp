#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std;
using namespace Eigen;

# define PI 3.14159265358979323846

class Timer
{
    bool clear = false;

    public:
    template<typename Function>
    void setTimeout(Function function, int delay);

    void stop();
};

void Timer::setTimeout(auto function, int delay)
{
    this->clear = false;
    std::thread t([=]()
    {
        if(this->clear) return;
        std::this_thread::sleep_for(std::chrono::microseconds(delay));
        if(this->clear) return;
        function();
    });
    t.detach();
}

void Timer::stop()
{
    this->clear = true;
}

class Interrupter : public rclcpp::Node
{
	public:

	key_t key = ftok("shmfile", 65);
	int shmid = shmget(key, 1024, 0666 | IPC_CREAT);
    int *ptr = (int*) shmat(shmid, (void*)0, 0);
    int state;
    Timer t;
	
	Interrupter() : Node("Interrupter")
	{
        state = 0;
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/icp/odom", 1, [this](nav_msgs::msg::Odometry::SharedPtr msg){ msg_callback(); });
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/interrupt_data", 1);
        // timer_ = this->create_wall_timer(1ms, [this]{ timer_callback(); });
	}

	private:

    void msg_callback() // const nav_msgs::msg::Odometry::SharedPtr msg_in)
    {
        int data = *(ptr + 1);
        auto drive_msg = std_msgs::msg::Int32();
        drive_msg.data = state;
        publisher_->publish(drive_msg);
        t.setTimeout([&]() {*(ptr + 1) == 0 ? *ptr = 1 : *ptr = 0}, 50000);
    }
	
	// void timer_callback()
	// {
	//  auto drive_msg = std_msgs::msg::Int32();
	//  int data = *(ptr + 1);
	//  drive_msg.data = data;
	//  publisher_->publish(drive_msg);
	// }

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
	
	rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Interrupter>());
  rclcpp::shutdown();
  return 0;
}
