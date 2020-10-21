#include <termios.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"


#define STEER_TOPIC "/steer_drive_controller/steering_position_controller/command"
#define DRIVE_TOPIC "/steer_drive_controller/wheel_drive_controller/command"

#define H (1)
#define W (2)


/* Function to read terminal inputs non-blocking (without ENTER being rpessed) */
int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt );             //save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                    // disable buffering 
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );    // apply new setting
	int ch = getchar();                           // read character (non-blocking)
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );    // restore old settings
	return ch;
}


int main(int argc, char **argv)
{
	float v = 0;
	double s = 0;
	double array[H][W] = {s, s};

	ros::init(argc, argv, "keyboard");
	ros::NodeHandle n;

	ros::Publisher front_R_vel = n.advertise<std_msgs::Float64>("/gotthard/steer_drive_controller/front_right_drive_controller/command", 1000);
	ros::Publisher front_L_vel = n.advertise<std_msgs::Float64>("/gotthard/steer_drive_controller/front_left_drive_controller/command", 1000);
	ros::Publisher rear_R_vel = n.advertise<std_msgs::Float64>("/gotthard/steer_drive_controller/rear_right_drive_controller/command", 1000);
	ros::Publisher rear_L_vel = n.advertise<std_msgs::Float64>("/gotthard/steer_drive_controller/rear_left_drive_controller/command", 1000);

	ros::Publisher key_steering = n.advertise<std_msgs::Float64MultiArray>("/gotthard/steer_drive_controller/steering_position_controller/command", 1000);
	ros::Rate loop_rate(5);

	std_msgs::Float64MultiArray drive_msg;
    std_msgs::Float64MultiArray steer_msg;

    drive_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    drive_msg.layout.dim[0].size = 4;
    drive_msg.layout.dim[0].stride = 1;

    steer_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    steer_msg.layout.dim[0].size = 2;
    steer_msg.layout.dim[0].stride = 1;

	std::string veh_name = "gotthard";
    ros::Publisher pub_steer = n.advertise<std_msgs::Float64MultiArray>("/" + veh_name + STEER_TOPIC, 1);
    ros::Publisher pub_drive = n.advertise<std_msgs::Float64MultiArray>("/" + veh_name + DRIVE_TOPIC, 1);

	std_msgs::Float64 velocity;
	std_msgs::Float64MultiArray steering;
	
	while (ros::ok())
	{ 
		std_msgs::Int32 key_press;

		steering.layout.dim.push_back(std_msgs::MultiArrayDimension());
		steering.layout.dim.push_back(std_msgs::MultiArrayDimension()); 
		steering.layout.dim[0].label = "height";
		steering.layout.dim[1].label = "width"; 
		steering.layout.dim[0].size = H;
		steering.layout.dim[1].size = W;
		steering.layout.dim[0].stride = H*W;
		steering.layout.dim[1].stride = W;
		steering.layout.data_offset = 0;
		std::vector<double>vec(W*H, 0); 
		
		key_press.data = getch();              // call your non-blocking input function

		if (key_press.data == 119) {                         // w: forward
			velocity.data = velocity.data + 5;
			ROS_INFO_STREAM("HI!");
			steering.data = vec;
			drive_msg.data.assign({velocity.data, velocity.data, velocity.data,velocity.data});
			pub_drive.publish(drive_msg);
		}
		else if (key_press.data == 115) {      // s: slower
			velocity.data = velocity.data - 5;
			steering.data = vec;
			drive_msg.data.assign({velocity.data, velocity.data, velocity.data,velocity.data});
			pub_drive.publish(drive_msg);

		}
		else if ((key_press.data == 97) && (s <= 1)) {       // a: left
			s = s + 0.1; 
			// for (int i=0; i<H; i++)
			// 	for (int j=0; j<W; j++)
			// 		vec[i*W+j] = s;
			// steering.data = vec;
			// key_steering.publish(steering);
			steer_msg.data.assign({s, s});
			pub_steer.publish(steer_msg);

		}
		else if ((key_press.data == 100) && (s >= -1)) {      // d: right
			s = s - 0.1; 
			// for (int i=0; i<H; i++)
			// 	for (int j=0; j<W; j++)
			// 		vec[i*W+j] = s;
			steer_msg.data.assign({s, s});
			pub_steer.publish(steer_msg);

		} 		
  		ros::spinOnce();
		loop_rate.sleep();
	}
}
