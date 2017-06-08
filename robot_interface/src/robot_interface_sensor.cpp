#include <iostream>
#include "eigen3/Eigen/Dense"
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
//#include "geometry_msgs/Twist.h"
#include <math.h>
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ContactState.h"
#include "std_msgs/Bool.h"
using Eigen::MatrixXd;
class feedback
{
private:
    ros::Publisher pub,move_robot;
    ros::Subscriber sub;

public:
    feedback(ros::NodeHandle nh);
    ~feedback()
    {
    }
    void forceCB(const gazebo_msgs::ContactsStateConstPtr &msg);
    void obtain_torque(MatrixXd end_force, MatrixXd &msg);

};
feedback::feedback(ros::NodeHandle nh)
{
    pub = nh.advertise<geometry_msgs::Wrench>("/haptics/torque",10);
    move_robot = nh.advertise<std_msgs::Bool>("/irb120/move_joints",10);
    sub = nh.subscribe("/irb120/force_feedback",10,&feedback::forceCB,this);
}
void feedback::obtain_torque(MatrixXd end_force, MatrixXd &msg)
{
    MatrixXd jacob(6,6);
    double q1,q2,q3,q4,q5,q6;
    q1 = 0;q2=0;q3=0;q4=0;q5=0;q6=0;

    jacob(0,0) = -3.0*((sin(q1)*cos(q4)*cos(q2 + q3) - sin(q4)*cos(q1))*cos(q5) - sin(q1)*sin(q5)*sin(q2 + q3))*cos(q6) + 3.0*(sin(q1)*sin(q4)*cos(q2 + q3) + cos(q1)*cos(q4))*sin(q6) - 132.1*sin(q1)*sin(q2)*cos(q3) - 132.1*sin(q1)*sin(q3)*cos(q2) - 132.1*sin(q1)*cos(q2);
    jacob(0,1) = (-132.1*sin(q2) + 3.0*sin(q4)*sin(q6)*sin(q2 + q3) - 3.0*sin(q5)*cos(q6)*cos(q2 + q3) - 3.0*sin(q2 + q3)*cos(q4)*cos(q5)*cos(q6) + 132.1*cos(q2 + q3))*cos(q1);
    jacob(0,2) =  (3.0*sin(q4)*sin(q6)*sin(q2 + q3) - 3.0*sin(q5)*cos(q6)*cos(q2 + q3) - 3.0*sin(q2 + q3)*cos(q4)*cos(q5)*cos(q6) + 132.1*cos(q2 + q3))*cos(q1);
    jacob(0,3) =  -3.0*(sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q2 + q3))*sin(q6) + 3.0*(sin(q1)*cos(q4) - sin(q4)*cos(q1)*cos(q2 + q3))*cos(q5)*cos(q6);
    jacob(0,4) =  -3.0*((sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q2 + q3))*sin(q5) + sin(q2 + q3)*cos(q1)*cos(q5))*cos(q6);
    jacob(0,5) =  -3.0*((sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q2 + q3))*cos(q5) - sin(q5)*sin(q2 + q3)*cos(q1))*sin(q6) + 3.0*(sin(q1)*cos(q4) - sin(q4)*cos(q1)*cos(q2 + q3))*cos(q6);

    jacob(1,0) = 3.0*((sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q2 + q3))*cos(q5) - sin(q5)*sin(q2 + q3)*cos(q1))*cos(q6) + 3.0*(sin(q1)*cos(q4) - sin(q4)*cos(q1)*cos(q2 + q3))*sin(q6) + 132.1*sin(q2)*cos(q1)*cos(q3) + 132.1*sin(q3)*cos(q1)*cos(q2) + 132.1*cos(q1)*cos(q2);
    jacob(1,1) =  (-132.1*sin(q2) + 3.0*sin(q4)*sin(q6)*sin(q2 + q3) - 3.0*sin(q5)*cos(q6)*cos(q2 + q3) - 3.0*sin(q2 + q3)*cos(q4)*cos(q5)*cos(q6) + 132.1*cos(q2 + q3))*sin(q1);
    jacob(1,2) =  (3.0*sin(q4)*sin(q6)*sin(q2 + q3) - 3.0*sin(q5)*cos(q6)*cos(q2 + q3) - 3.0*sin(q2 + q3)*cos(q4)*cos(q5)*cos(q6) + 132.1*cos(q2 + q3))*sin(q1);
    jacob(1,3) =  -3.0*(sin(q1)*sin(q4)*cos(q2 + q3) + cos(q1)*cos(q4))*cos(q5)*cos(q6) - 3.0*(sin(q1)*cos(q4)*cos(q2 + q3) - sin(q4)*cos(q1))*sin(q6);
    jacob(1,4) =  -3.0*((sin(q1)*cos(q4)*cos(q2 + q3) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*sin(q2 + q3)*cos(q5))*cos(q6);
    jacob(1,5) =  -3.0*((sin(q1)*cos(q4)*cos(q2 + q3) - sin(q4)*cos(q1))*cos(q5) - sin(q1)*sin(q5)*sin(q2 + q3))*sin(q6) - 3.0*(sin(q1)*sin(q4)*cos(q2 + q3) + cos(q1)*cos(q4))*cos(q6);

    jacob(2,0) = 0;
    jacob(2,1) = -3.0*sin(q4)*sin(q6)*cos(q2 + q3) - 3.0*sin(q5)*sin(q2 + q3)*cos(q6) + 132.1*sin(q2 + q3) + 132.1*cos(q2) + 3.0*cos(q4)*cos(q5)*cos(q6)*cos(q2 + q3);
    jacob(2,2) =  -3.0*sin(q4)*sin(q6)*cos(q2 + q3) - 3.0*sin(q5)*sin(q2 + q3)*cos(q6) + 132.1*sin(q2 + q3) + 3.0*cos(q4)*cos(q5)*cos(q6)*cos(q2 + q3);
    jacob(2,3) = -3.0*(sin(q4)*cos(q5)*cos(q6) + sin(q6)*cos(q4))*sin(q2 + q3);
    jacob(2,4) =  3.0*(-sin(q5)*sin(q2 + q3)*cos(q4) + cos(q5)*cos(q2 + q3))*cos(q6);
    jacob(2,5) =  -3.0*sin(q4)*sin(q2 + q3)*cos(q6) - 3.0*sin(q5)*sin(q6)*cos(q2 + q3) -3.0*sin(q6)*sin(q2 + q3)*cos(q4)*cos(q5);

    jacob(3,0) = sin(q1);
    jacob(3,1) = sin(q1);
    jacob(3,2) = sin(q2 + q3)*cos(q1);
    jacob(3,3) =  sin(q1)*cos(q4) - sin(q4)*cos(q1)*cos(q2 + q3);
    jacob(3,4) = (sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q2 + q3))*sin(q5) + sin(q2 + q3)*cos(q1)*cos(q5);
    jacob(3,5) = (sin(q1)*sin(q4) + cos(q1)*cos(q4)*cos(q2 + q3))*sin(q5) + sin(q2 + q3)*cos(q1)*cos(q5);

    jacob(4,0) = -cos(q1);
    jacob(4,1) =  -cos(q1);
    jacob(4,2) = sin(q1)*sin(q2 + q3);
    jacob(4,3) = -sin(q1)*sin(q4)*cos(q2 + q3) - cos(q1)*cos(q4);
    jacob(4,4) = (sin(q1)*cos(q4)*cos(q2 + q3) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*sin(q2 + q3)*cos(q5);
    jacob(4,5) = (sin(q1)*cos(q4)*cos(q2 + q3) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*sin(q2 + q3)*cos(q5);

    jacob(5,0) = 0;
    jacob(5,1) = 0;
    jacob(5,2) = -cos(q2 + q3);
    jacob(5,3) = -sin(q4)*sin(q2 + q3);
    jacob(5,4) =  sin(q5)*sin(q2 + q3)*cos(q4) - cos(q5)*cos(q2 + q3);
    jacob(5,5) =  sin(q5)*sin(q2 + q3)*cos(q4) - cos(q5)*cos(q2 + q3);

    for (int i = 0; i <6; i++)
         for(int j=0;j<6;j++)
             jacob(i,j) = floorf(jacob(i,j)*10000)/10000;
    std::cout<<"Jacobian\n"<< jacob << std::endl;

    msg = jacob.transpose()*end_force;

    std::cout<<"Torque\n"<< msg<<std::endl;


}

void feedback::forceCB(const gazebo_msgs::ContactsStateConstPtr &msg)
{
    MatrixXd end_force(6,1);
    gazebo_msgs::ContactState wrench_data;
    geometry_msgs::Wrench ft_msg,feedback_msg;
    MatrixXd torque(6,1);
    std_msgs::Bool move_robot_val;
    if(msg->states.size())
    {
        wrench_data = msg->states[0];
        ft_msg = wrench_data.wrenches[0];
        end_force(0,0) = ft_msg.force.x;
        end_force(1,0) = ft_msg.force.y;
        end_force(2,0) = ft_msg.force.z;
        end_force(3,0) = ft_msg.torque.x;
        end_force(4,0) = ft_msg.torque.y;
        end_force(5,0) = ft_msg.torque.z;
        ROS_INFO("seq: %d  X:%.2f Y:%.2f Z:%.2f",msg->header.seq,end_force(0,0),end_force(1,0),end_force(2,0));
        obtain_torque(end_force, torque );
//        move_robot.publish

        feedback_msg.torque.x = torque(0,0);
        feedback_msg.torque.y = torque(1,0);
        feedback_msg.torque.z = torque(2,0);

        feedback_msg.force.x = torque(3,0);
        feedback_msg.force.y = torque(4,0);
        feedback_msg.force.z = torque(5,0);

        pub.publish(feedback_msg);

    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"sensor_node");
    ros::NodeHandle nh;
    feedback obj(nh);
    ros::Rate freq(10); // 10 hz
    while(ros::ok())
    {
        ros::spinOnce();
        freq.sleep();
    }
    return 0;
}
