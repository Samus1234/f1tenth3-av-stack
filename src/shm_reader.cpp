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
using namespace Eigen;

# define PI 3.14159265358979323846

class SHMRead : public rclcpp::Node
{
	public:

	key_t key = ftok("/home/schmidd/f1tenth_ws/src/f1tenth3-av-stack/src/shmfile", 65);
	int shmid = shmget(key, 1024, 0666 | IPC_CREAT);
	int *ptr = (int*) shmat(shmid, (void*)0, 0);
	int state;
	bool edge_detect;
	
	SHMRead() : Node("SHMRead")
	{
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/read_data", 1);
        timer_ = this->create_wall_timer(100ms, [this]{ timer_callback(); });
	}

	private:
	
	void timer_callback()
	{
		int shr_data = *ptr;
		auto drive_msg = std_msgs::msg::Int32();
		edge_detect = ~state & shr_data;
		drive_msg.data = (int) edge_detect;
		publisher_->publish(drive_msg);
		state = shr_data;
	}

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
	
	rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SHMRead>());
  rclcpp::shutdown();
  return 0;
}
