#include <iostream>
#include "eigen3/Eigen/Dense"
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"


#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include <pthread.h>
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ContactState.h"
#include "std_msgs/Bool.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <sstream>

#include "geomagic_control/PhantomButtonEvent.h"
#include "geomagic_control/OmniFeedback.h"


using Eigen::MatrixXd;
HDdouble joint_angles[3];
hduVector3Dd gimbal_angles;
MatrixXd joint_states(6,1);
HDdouble joint_torque[3];
HDdouble velocity[3];
HDdouble ang_velocity[3];
MatrixXd velocity_m(6,1);
int calibrationStyle;
struct OmniState {
	hduVector3Dd position;  //3x1 vector of position
	hduVector3Dd velocity;  //3x1 vector of velocity
	hduVector3Dd inp_vel1; //3x1 history of velocity used for filtering velocity estimate
	hduVector3Dd inp_vel2;
	hduVector3Dd inp_vel3;
	hduVector3Dd out_vel1;
	hduVector3Dd out_vel2;
	hduVector3Dd out_vel3;
	hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
	hduVector3Dd pos_hist2;
	hduVector3Dd rot;
	hduVector3Dd joints;
	hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
	float thetas[7];
	int buttons[2];
	int buttons_prev[2];
	int lock[3];
	hduVector3Dd lock_pos;
};
void init(OmniState *s) {
    OmniState *state;
		state = s;
		state->buttons[0] = 0;
		state->buttons[1] = 0;
		state->buttons_prev[0] = 0;
		state->buttons_prev[1] = 0;
		hduVector3Dd zeros(0, 0, 0);
		state->velocity = zeros;
		state->inp_vel1 = zeros;  //3x1 history of velocity
		state->inp_vel2 = zeros;  //3x1 history of velocity
		state->inp_vel3 = zeros;  //3x1 history of velocity
		state->out_vel1 = zeros;  //3x1 history of velocity
		state->out_vel2 = zeros;  //3x1 history of velocity
		state->out_vel3 = zeros;  //3x1 history of velocity
		state->pos_hist1 = zeros; //3x1 history of position
		state->pos_hist2 = zeros; //3x1 history of position
		state->lock[0] = false;
		state->lock[1] = false;
		state->lock[2] = false;
		state->lock_pos = zeros;
	}
class feedback
{
private:
    ros::Publisher pub,move_robot,pub1;
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
    pub1 = nh.advertise<geometry_msgs::Twist>("/haptics/torque",10);

    sub = nh.subscribe("/irb120/force_feedback",10,&feedback::forceCB,this);
}
void feedback::obtain_torque(MatrixXd end_force, MatrixXd &msg)
{
    MatrixXd jacob(6,6);
    double q1,q2,q3,q4,q5,q6;
    q1 = joint_states(0,0);
    q2 = joint_states(1,0);
    q3 = joint_states(2,0);
    q4 = joint_states(3,0);
    q5 = joint_states(4,0);
    q6 = joint_states(5,0);

    jacob(0,0) = 30.0*sin(q1)*sin(q5)*cos(q2 + q3) - 30.0*sin(q1)*sin(q2 + q3)*cos(q4)*cos(q5) + 132.1*sin(q1)*cos(q2) + 132.1*sin(q1)*cos(q2 + q3) + 30.0*sin(q4)*cos(q1)*cos(q5);
    jacob(0,1) = (132.1*sin(q2) + 30.0*sin(q5)*sin(q2 + q3) + 132.1*sin(q2 + q3) + 30.0*cos(q4)*cos(q5)*cos(q2 + q3))*cos(q1);
    jacob(0,2) =  (30.0*sin(q5)*sin(q2 + q3) + 132.1*sin(q2 + q3) + 30.0*cos(q4)*cos(q5)*cos(q2 + q3))*cos(q1);
    jacob(0,3) =   30.0*(sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q5);
    jacob(0,4) =  -30.0*sin(q1)*sin(q4)*sin(q5) - 30.0*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 30.0*cos(q1)*cos(q5)*cos(q2 + q3);
    jacob(0,5) =  0;

    jacob(1,0) = 30.0*sin(q1)*sin(q4)*cos(q5) - 30.0*sin(q5)*cos(q1)*cos(q2 + q3) + 30.0*sin(q2 + q3)*cos(q1)*cos(q4)*cos(q5) - 132.1*cos(q1)*cos(q2) - 132.1*cos(q1)*cos(q2 + q3);
    jacob(1,1) =  (132.1*sin(q2) + 30.0*sin(q5)*sin(q2 + q3) + 132.1*sin(q2 + q3) + 30.0*cos(q4)*cos(q5)*cos(q2 + q3))*sin(q1);
    jacob(1,2) =   (30.0*sin(q5)*sin(q2 + q3) + 132.1*sin(q2 + q3) + 30.0*cos(q4)*cos(q5)*cos(q2 + q3))*sin(q1);
    jacob(1,3) =   -30.0*(sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q5);
    jacob(1,4) =  -30.0*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 30.0*sin(q1)*cos(q5)*cos(q2 + q3) + 30.0*sin(q4)*sin(q5)*cos(q1);
    jacob(1,5) =  0;

    jacob(2,0) = 0;
    jacob(2,1) = -30.0*sin(q5)*cos(q2 + q3) + 30.0*sin(q2 + q3)*cos(q4)*cos(q5) - 132.1*cos(q2) - 132.1*cos(q2 + q3);
    jacob(2,2) =  -30.0*sin(q5)*cos(q2 + q3) + 30.0*sin(q2 + q3)*cos(q4)*cos(q5) - 132.1*cos(q2 + q3);
    jacob(2,3) = 30.0*sin(q4)*cos(q5)*cos(q2 + q3);
    jacob(2,4) = 30.0*sin(q5)*cos(q4)*cos(q2 + q3) - 30.0*sin(q2 + q3)*cos(q5);
    jacob(2,5) = 0;

    jacob(3,0) = sin(q1);
    jacob(3,1) = sin(q1);
    jacob(3,2) = -cos(q1)*cos(q2 + q3);
    jacob(3,3) =  -sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1);
    jacob(3,4) =  (sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) - sin(q5)*cos(q1)*cos(q2 + q3);
    jacob(3,5) =  (sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) - sin(q5)*cos(q1)*cos(q2 + q3);

    jacob(4,0) = -cos(q1);
    jacob(4,1) =  -cos(q1);
    jacob(4,2) = -sin(q1)*cos(q2 + q3);
    jacob(4,3) = sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4);
    jacob(4,4) = (sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) - sin(q1)*sin(q5)*cos(q2 + q3);
    jacob(4,5) =  (sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) - sin(q1)*sin(q5)*cos(q2 + q3);

    jacob(5,0) = 0;
    jacob(5,1) = 0;
    jacob(5,2) = -sin(q2 + q3);
    jacob(5,3) = -sin(q4)*cos(q2 + q3);
    jacob(5,4) =  -sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3);
    jacob(5,5) = -sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3);

    for (int i = 0; i <6; i++)
         for(int j=0;j<6;j++)
             jacob(i,j) = floorf(jacob(i,j)*10000)/10000;
    msg = jacob.transpose()*end_force;

}

void feedback::forceCB(const gazebo_msgs::ContactsStateConstPtr &msg)
{
    MatrixXd end_force(6,1);
    gazebo_msgs::ContactState wrench_data;
    geometry_msgs::Wrench ft_msg,feedback_msg;
    geometry_msgs::Twist vel_msg;
    MatrixXd torque(6,1);
    std_msgs::Bool move_robot_val;
    HDdouble velocity[3];
    HDdouble ang_velocity[3];
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
        obtain_torque(end_force, torque);
//        move_robot.publish

        feedback_msg.torque.x = torque(0,0);
        feedback_msg.torque.y = torque(1,0);
        feedback_msg.torque.z = torque(2,0);

        feedback_msg.force.x = torque(3,0);
        feedback_msg.force.y = torque(4,0);
        feedback_msg.force.z = torque(5,0);

        vel_msg.linear.x = velocity_m(0,0);
        vel_msg.linear.y = velocity_m(1,0);
        vel_msg.linear.z = velocity_m(2,0);

        vel_msg.angular.x = velocity_m(3,0);
        vel_msg.angular.y = velocity_m(4,0);
        vel_msg.angular.z = velocity_m(5,0);

        // converting from Nmm to Nm and multiplying by a factor
        joint_torque[0] = torque(0,0)/20;
        joint_torque[1] = torque(1,0)/20;
        joint_torque[2] = torque(2,0)/20;

        ROS_INFO("Joint Torques: %f %f %f",joint_torque[0],joint_torque[1],joint_torque[2]);

        pub.publish(feedback_msg);
        pub1.publish(vel_msg);


    }
}

HDCallbackCode HDCALLBACK gravityWellCallback(void *data)
{

    HHD hHD = hdGetCurrentDevice();
    HDdouble position[3];

    HDdouble joint_angles[3];
    hdBeginFrame(hHD);

    hdGetDoublev(HD_CURRENT_JOINT_ANGLES,joint_angles);
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);


//    ROS_INFO("angle x %.2f", joint_angles[0]);
    joint_states(0,0) = joint_angles[0];
    joint_states(1,0) = joint_angles[1];
    joint_states(2,0) = joint_angles[2];
    joint_states(3,0) = gimbal_angles[0];
    joint_states(4,0) = gimbal_angles[1];
    joint_states(5,0) = gimbal_angles[2];
    hdEndFrame(hHD);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during scheduler callback");
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }
    //hdSetDoublev(HD_CURRENT_JOINT_TORQUE, joint_torque);
    return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) {
	OmniState *omni_state = static_cast<OmniState *>(pUserData);
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
	  ROS_DEBUG("Updating calibration...");
	    hdUpdateCalibration(calibrationStyle);
	  }
	hdBeginFrame(hdGetCurrentDevice());
	//Get angles, set forces
	ros::Publisher pub1;
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);
	hdGetDoublev(HD_CURRENT_POSITION, omni_state->position);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES,joint_angles);
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
    hdGetDoublev(HD_CURRENT_VELOCITY,velocity);
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY,ang_velocity);

    velocity_m(0,0) = velocity[0];
    velocity_m(1,0) = velocity[1];
    velocity_m(2,0) = velocity[2];
    velocity_m(3,0) = ang_velocity[0];
    velocity_m(4,0) = ang_velocity[1];
    velocity_m(5,0) = ang_velocity[2];

	joint_states(0,0) = joint_angles[0];
    joint_states(1,0) = joint_angles[1];
    joint_states(2,0) = joint_angles[2];
    joint_states(3,0) = gimbal_angles[0];
    joint_states(4,0) = gimbal_angles[1];
    joint_states(5,0) = gimbal_angles[2];


//	hdGetDoublev(HD_CURRENT_JOINT_ANGLES,joint_angles);
//	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
//    simplify_jacobian(joint_angles[0],joint_angles[1],joint_angles[2],gimbal_angles[0],gimbal_angles[1],gimbal_angles[2]);


	hduVector3Dd vel_buff(0, 0, 0);
	vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
			+ omni_state->pos_hist2) / 0.002;  //mm/s, 2nd order backward dif
	omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
			+ .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
			- (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
					- 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
	omni_state->pos_hist2 = omni_state->pos_hist1;
	omni_state->pos_hist1 = omni_state->position;
	omni_state->inp_vel3 = omni_state->inp_vel2;
	omni_state->inp_vel2 = omni_state->inp_vel1;
	omni_state->inp_vel1 = vel_buff;
	omni_state->out_vel3 = omni_state->out_vel2;
	omni_state->out_vel2 = omni_state->out_vel1;
	omni_state->out_vel1 = omni_state->velocity;
	for(int i=0; i<3;i++){
		if (omni_state->lock[i]) {
			omni_state->force[i] = 0.3 * (omni_state->lock_pos[i] - omni_state->position[i])
				- 0.001 * omni_state->velocity[i];
		}
	}
//	ROS_INFO("force x %.2f", joint_angles(0));
//	ROS_INFO("force y %.2f" , omni_state->force[1]);

	hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);
//    HDdouble baseTorque[3] = {75,100,100}; //Base Torque in mNm
    hdSetDoublev(HD_CURRENT_JOINT_TORQUE, joint_torque);
	//Get buttons
	int nButtons = 0;
	hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
	omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
	omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

	hdEndFrame(hdGetCurrentDevice());

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Error during main scheduler callback");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}

	float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
			omni_state->joints[2] - omni_state->joints[1], omni_state->rot[0],
			omni_state->rot[1], omni_state->rot[2] };
	for (int i = 0; i < 7; i++)
		omni_state->thetas[i] = t[i];
	return HD_CALLBACK_CONTINUE;
}


void HHD_Auto_Calibration() {
	int supportedCalibrationStyles;
	HDErrorInfo error;

	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		ROS_INFO("HD_CALIBRATION_ENCODER_RESE..");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
		calibrationStyle = HD_CALIBRATION_INKWELL;
		ROS_INFO("HD_CALIBRATION_INKWELL..");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
		calibrationStyle = HD_CALIBRATION_AUTO;
		ROS_INFO("HD_CALIBRATION_AUTO..");
	}
	if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
	  do {
		hdUpdateCalibration(calibrationStyle);
		ROS_INFO("Calibrating.. (put stylus in well)");
		if (HD_DEVICE_ERROR(error = hdGetError())) {
			hduPrintError(stderr, &error, "Reset encoders reset failed.");
			break;
		}
	} while (hdCheckCalibration() != HD_CALIBRATION_OK);
	ROS_INFO("Calibration complete.");
	}
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
	  ROS_INFO("Please place the device into the inkwell for calibration.");
	}
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"calc_joint_torque");
    ros::NodeHandle nh;

    feedback obj(nh);
    ros::Rate freq(100);

    HDErrorInfo error;
    HDSchedulerHandle hGravityWell;
	HHD hHD;
	hHD = hdInitDevice("HD1");//use ros param and set in launch file
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		//hduPrintError(stderr, &error, "Failed to initialize haptic device");
		ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
		return -1;
	}

	ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		ROS_ERROR("Failed to start the scheduler"); //, &error);
		return -1;
	}
    HHD_Auto_Calibration();


    /*hGravityWell = hdScheduleAsynchronous(
        gravityWellCallback, 0,
        HD_MAX_SCHEDULER_PRIORITY);
*/
    OmniState state;
    init(&state);
	hdScheduleAsynchronous(omni_state_callback, &state,
			HD_MAX_SCHEDULER_PRIORITY);




     // 100 hz
    while(ros::ok())
    {
        ros::spinOnce();
        freq.sleep();
    }

    ROS_INFO("Ending Session....");
	hdStopScheduler();
	hdDisableDevice(hHD);

    return 0;
}
