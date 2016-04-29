/* Laughlin Barker | 2016
 * laughlin@jhu.edu
 * Written for EN 530.707 | Robot Systems Programming | Johns Hopkins University
 */

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Joy.h>

#include <std_msgs/Int32.h>
#include "std_msgs/Int32MultiArray.h"
#include <std_msgs/Float32.h>

#include <eigen/Eigen/Core>
#include <eigen/unsupported/Eigen/MatrixFunctions>

class OpenROVTeleop
{
public:
    OpenROVTeleop();

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh;

    int x_controllerAxis, z_controllerAxis, yaw_controllerAxis; //joy.axis indicies for axis for respective movements
    int lightsAdj, laserToggle, camTilt;   //joy.buttons indicies for ROV commands
    double x_gain, z_gain, yaw_gain;    //gain for respective movements
    std_msgs::Int32MultiArray motor_cmds[3];       //array of motors commands [stbd, port, vert]'

    //published topics - future improvement: custom OpenROV msg that contains all
    ros::Publisher motorPub;
    ros::Publisher lightPub;
    ros::Publisher laserPub;
    ros::Publisher camTiltPub;

    ros::Subscriber joySub;

    double d;       //thruster distance from center line

    Eigen::Matrix3d A;
};

OpenROVTeleop::OpenROVTeleop():
    //controller axis/button mapping can be found here: http://wiki.ros.org/joy
    x_controllerAxis(1),       //left stick up/down
    z_controllerAxis(3),       //right stick up/down
    yaw_controllerAxis(0),      //left stick left/right
    lightsAdj(6),   // cross key left/right
    laserToggle(10),    //button stick right
    camTilt(7),      // cross key up/down
    x_gain(1),
    z_gain(1),
    yaw_gain(1)
{
    //load settings from parameter server (if availiable), overwrite defaults
    nh.param("X_stick", x_controllerAxis, x_controllerAxis);
    nh.param("Z_stick", z_controllerAxis, z_controllerAxis);
    nh.param("Yaw_stick", yaw_controllerAxis, yaw_controllerAxis);
    nh.param("lights_adj",lightsAdj, lightsAdj);
    nh.param("laser_tottle", laserToggle, laserToggle);
    nh.param("camera_tilt", camTilt, camTilt);
    nh.param("x_gain", x_gain, x_gain);
    nh.param("z_gain", z_gain, z_gain);
    nh.param("yaw_gain", yaw_gain, yaw_gain);


    //initialize publishers and subscribers
    joySub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &OpenROVTeleop::joyCallback, this);

    motorPub = nh.advertise<std_msgs::Int32MultiArray>("/openrov/motortarget", 1);
    lightPub = nh.advertise<std_msgs::Float32>("/openrov/light_command", 1);
    laserPub = nh.advertise<std_msgs::Int32>("/openrov/laser_toggle", 1);
    camTiltPub = nh.advertise<std_msgs::Int32>("/openrov/camera_servo",1);
}

void OpenROVTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double fx_d, fz_d, mz_d; //desired forces in x,z and torque about z
    d = 0.045;       // [m] - horizontal thruster distance from centerline of CG


    // our job now is to calcualte desired prop speed, given some input.
    // in the short-term let us interpret joystick inputs as a desired wrench (force/torque)
    // instead of a twist (linear/angular velocity - more intuitive), dynamics can come later

    fx_d = x_gain * joy->axes[x_controllerAxis];
    fz_d = z_gain * joy->axes[z_controllerAxis];
    mz_d = yaw_gain * joy->axes[yaw_controllerAxis];

    // thruster allocation matrix
    A << 1, 1, 0,
         0, 0, 1,
         d, -d, 0;

    // ROV body frame forces/torque vector [fx, fz, mz]' - marine body conventions: x: forward, y: stdb, z: down
    Eigen::Vector3d F(fx_d,fz_d,mz_d);

    // solve for thruster force vector [T_stbd, T_port, T_vert]'
    Eigen::Vector3d T =  A.inverse() * F;  // A.inv is safe because A full rank <--> invertiable

    //deal with thruster saturation

    //calculate required ESC command to achieve required motor force

    //publish ESC command to /openrov/motor_cmd topic

}

double computeThrustGraupner230860(int pctThrust)
{
double t;

return t;
}

double computeThrustGraupner230357(int pctThrust)
{
double t;

return t;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "openrov_teleop");

    OpenROVTeleop rovTeleop;

    ros::spin();
}
