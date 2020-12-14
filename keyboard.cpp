#include <termios.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#define steer_H (1)
#define steer_W (2)

#define wheel_drive_H (1)
#define wheel_drive_W (4)


/* Function to read terminal inputs non-blocking (without ENTER being pressed) */
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

	ros::init(argc, argv, "keyboard");
	ros::NodeHandle n;

	ros::Publisher key_steering = n.advertise<std_msgs::Float64MultiArray>("/19E/steer_drive_controller/steering_position_controller/command", 1000);

	ros::Publisher key_wheel_drive = n.advertise<std_msgs::Float64MultiArray>("/19E/steer_drive_controller/wheel_drive_controller/command", 1000);
	ros::Rate loop_rate(5);

	
	while (ros::ok())
	{ 
		std_msgs::Int32 key_press;
		std_msgs::Float64 velocity;
		std_msgs::Float64MultiArray steering;
		std_msgs::Float64MultiArray wheel_drive;

		steering.layout.dim.push_back(std_msgs::MultiArrayDimension());
		steering.layout.dim.push_back(std_msgs::MultiArrayDimension()); 
		steering.layout.dim[0].label = "height";
		steering.layout.dim[1].label = "width"; 
		steering.layout.dim[0].size = steer_H;
		steering.layout.dim[1].size = steer_W;
		steering.layout.dim[0].stride = steer_H*steer_W;
		steering.layout.dim[1].stride = steer_W;
		steering.layout.data_offset = 0;

		wheel_drive.layout.dim.push_back(std_msgs::MultiArrayDimension());
		wheel_drive.layout.dim.push_back(std_msgs::MultiArrayDimension()); 
		wheel_drive.layout.dim[0].label = "height";
		wheel_drive.layout.dim[1].label = "width"; 
		wheel_drive.layout.dim[0].size = wheel_drive_H;
		wheel_drive.layout.dim[1].size = wheel_drive_W;
		wheel_drive.layout.dim[0].stride = wheel_drive_H*wheel_drive_W;
		wheel_drive.layout.dim[1].stride = wheel_drive_W;
		wheel_drive.layout.data_offset = 0;

		std::vector<double>steer_vec(steer_W*steer_H, 0); 
		std::vector<double>wheel_drive_vec(wheel_drive_H*wheel_drive_W, 0);
		
		key_press.data = getch();              // call your non-blocking input function

		if (key_press.data == 119) {                         // w: forward

			for (int i=0; i<wheel_drive_H; i++)
				for (int j=0; j<wheel_drive_W; j++)
					wheel_drive_vec[i*wheel_drive_W+j] = 4;
			wheel_drive.data = wheel_drive_vec;
			key_wheel_drive.publish(wheel_drive);
		}
		else if (key_press.data == 115) {      // s: slower

			for (int i=0; i<wheel_drive_H; i++)
				for (int j=0; j<wheel_drive_W; j++)
					wheel_drive_vec[i*wheel_drive_W+j] = -4;
			wheel_drive.data = wheel_drive_vec;
			key_wheel_drive.publish(wheel_drive);
		}
		else {
			for (int i=0; i<wheel_drive_H; i++)
				for (int j=0; j<wheel_drive_W; j++)
					wheel_drive_vec[i*wheel_drive_W+j] = 0;
			wheel_drive.data = wheel_drive_vec;
			key_wheel_drive.publish(wheel_drive);
		}


		if ((key_press.data == 97) && (s <= 1)) {       // a: left
			s = s + 0.1; 
			for (int i=0; i<steer_H; i++)
				for (int j=0; j<steer_W; j++)
					steer_vec[i*steer_W+j] = s;
			steering.data = steer_vec;
			key_steering.publish(steering);
		}
		else if ((key_press.data == 100) && (s >= -1)) {      // d: right
			s = s - 0.1; 
			for (int i=0; i<steer_H; i++)
				for (int j=0; j<steer_W; j++)
					steer_vec[i*steer_W+j] = s;
			steering.data = steer_vec; 
			key_steering.publish(steering);
		}
 		

  		ros::spinOnce();
		loop_rate.sleep();
	}
}
