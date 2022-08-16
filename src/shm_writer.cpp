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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std;

class SHMWrite : public rclcpp::Node
{
	public:

	key_t key = ftok("/home/schmidd/f1tenth_ws/src/f1tenth3-av-stack/src/shmfile", 65);
	int shmid = shmget(key, 1024, 0666 | IPC_CREAT);
	int *ptr = (int*) shmat(shmid, (void*)0, 0);
    int state = 0;
	
	SHMWrite() : Node("SHMWrite")
	{
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/write_data", 1);
        timer_ = this->create_wall_timer(1000ms, [this]{ timer_callback(); });
	}

	private:
	
	void timer_callback()
	{
		*ptr = state;
		// *(ptr + 1) = state;
		auto drive_msg = std_msgs::msg::Int32();
		drive_msg.data = state;
		publisher_->publish(drive_msg);
		state ^= 1;
	}

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
	
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SHMWrite>());
  rclcpp::shutdown();
  return 0;
}
