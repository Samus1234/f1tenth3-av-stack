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
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std;
using namespace Eigen;

# define PI 3.14159265358979323846

key_t key = ftok("/home/schmidd/f1tenth_ws/src/f1tenth3-av-stack/src/shmfile", 65);
int shmid = shmget(key, 1024, 0666 | IPC_CREAT);
int *ptr = (int*) shmat(shmid, (void*)0, 0);

class Timer
{
    public:

        typedef std::function<void(void)> Function;
    
        bool clear = false;

        Timer() : beg_(clock_::now()) {}

        void reset() { beg_ = clock_::now(); }

        double elapsed() const { return std::chrono::duration_cast<second_>(clock_::now() - beg_).count(); }

        void start(const Function &function, float delay)
        {
            if(clear) return;

            th = std::thread([=]()
            {
                reset();

                while (elapsed() < delay)
                {
                    if(clear) return;
                }

                clear = true;

                function();
            });

            th.detach();
        }

    private:

        typedef std::chrono::high_resolution_clock clock_;

        typedef std::chrono::duration<double, std::ratio<1> > second_;

        std::thread th;
        
        std::chrono::time_point<clock_> beg_;
};

class Interrupter : public rclcpp::Node
{
	public:
    int interrupt_time, thread_count;
    int icp_run_state;
    float t_delay_;
    std_msgs::msg::Int32 pub_msg_;
    Timer t;
	
	Interrupter() : Node("Interrupter")
	{
        declare_parameter("interrupt_time", 1000);
        t_delay_ = ((float) get_parameter("interrupt_time").as_int())*1e-6;
        thread_count = 0;
        icp_run_state = 0;
        *ptr = 0;
        subscriber_start_ = this->create_subscription<std_msgs::msg::Int32>("/icp_state", 1, [this](std_msgs::msg::Int32::SharedPtr msg){ start_cb(msg); });
        subscriber_end_ = this->create_subscription<nav_msgs::msg::Odometry>("/icp/odom", 1, [this](nav_msgs::msg::Odometry::SharedPtr msg){ end_cb(); });
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/interrupt_data", 1);
        timer_ = this->create_wall_timer(10ms, [this]{ timer_callback(); });
	}

	private:

    // void start_cb(const std_msgs::msg::Int32::SharedPtr state_msg)
    // {
    //     // int data_icp = state_msg.get()->data;

    //     icp_run_state = 1;

    //     // pub_msg_.data = icp_run_state;

    //     // publisher_ -> publish(pub_msg_);

    //     // t.clear = true;

    //     // std_msgs::msg::Int32 rising;

    //     // t.clear = false;
    //     // t.start(
    //     //     [&]()
    //     //     {
    //     //         *ptr ^= 1;
    //     //         thread_count++;
    //     //     }, t_delay_
    //     // );

    //     // rising.data = thread_count;
    //     // publisher_ -> publish(rising);
    // }

    // void end_cb()
    // {
    //     // int data_icp = state_msg.get()->data;

    //     icp_run_state = 0;

    //     // pub_msg_.data = icp_run_state;

    //     // publisher_ -> publish(pub_msg_);

    // }

	void timer_callback()
	{
        pub_msg_.data = icp_run_state;
        publisher_ -> publish(pub_msg_);
	}

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_start_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_end_;
	
	rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Interrupter>());
  rclcpp::shutdown();
  return 0;
}
