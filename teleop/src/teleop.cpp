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
#include <openrov/motortarget.h>
#include <std_msgs/Float32.h>

#include <eigen/Eigen/Core>
#include <eigen/unsupported/Eigen/MatrixFunctions>

class OpenROVTeleop
{
public:
    OpenROVTeleop();

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    double limitThrusterSaturation(double &Ppct_d, double &Vpct_d, double &Spct_d);
    double computePctThrustGraupner230860(double &fDes);
    double computePctThrustGraupner230357(double &fDes);


    ros::NodeHandle nh;

    int x_controllerAxis, z_controllerAxis, yaw_controllerAxis; //joy.axis indicies for axis for respective movements
    int lightsAdj, laserToggle, camTilt;   //joy.buttons indicies for ROV commands
    double x_gain, z_gain, yaw_gain;    //gain for respective movements
    openrov::motortarget motor_cmds;       //array of motors commands [port, vert, stbd]'

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

    motorPub = nh.advertise<openrov::motortarget>("/openrov/motortarget", 1);
    lightPub = nh.advertise<std_msgs::Float32>("/openrov/light_command", 1);
    laserPub = nh.advertise<std_msgs::Int32>("/openrov/laser_toggle", 1);
    camTiltPub = nh.advertise<std_msgs::Int32>("/openrov/camera_servo",1);
}

void OpenROVTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double fx_d, fz_d, mz_d; //desired forces in x,z and torque about z
    d = 0.045;       // [m] - horizontal thruster distance from centerline of CG


    // our job now is to calcualte desired prop speed (or pct thrust for PWM ESCs) , given some input.
    // in the short-term let us interpret joystick inputs as a desired wrench (force/torque)
    // instead of a twist (linear/angular velocity - more intuitive), dynamics can come later

    fx_d = x_gain * joy->axes[x_controllerAxis];
    fz_d = z_gain * joy->axes[z_controllerAxis];
    mz_d = yaw_gain * joy->axes[yaw_controllerAxis];

    // thruster allocation matrix
    A << 1, 0, 1,
         0, 1, 0,
         -d, 0, d;      //full rank

    // ROV body frame forces/torques [fx, fz, mz]' - marine body conventions: x: forward, y: stdb, z: down
    Eigen::Vector3d F(fx_d,fz_d,mz_d);

    // solve for thruster force vector [T_port, T_vert, T_stbd]'
    Eigen::Vector3d T =  A.inverse() * F;  // A.inv safe because A full rank (by inspection, above) <--> invertiable


    //calculate desired percentage thrust from each thruster
    double Ppct_d, Vpct_d, Spct_d;
    Ppct_d = computePctThrustGraupner230860(T(0));
    Vpct_d = computePctThrustGraupner230357(T(1));
    Spct_d = computePctThrustGraupner230860(T(2));

    //now we deal with thruster saturation - ROV pilots often prefer prioritizing heading authority
    //but for now lets just scale everything to bring within saturation limits
    //dirty hack using below hard coded value
    double mz_max = 2 * d * 7.35;      // [N] see below for reference
    double fx_max_fwd = 2 * 14.7;
    double fx_max_rev = 2 * 7.35;

    double scaleFactor = OpenROVTeleop::limitThrusterSaturation(Ppct_d, Vpct_d, Spct_d);

}

//check for thruster saturation, and if found returns scale thrust vector to avoid said saturation
double OpenROVTeleop::limitThrusterSaturation(double &Ppct_d, double &Vpct_d, double &Spct_d)
{
    double max, min;
    //find maximum desired thrust percentage
    max = std::max(Ppct_d,Vpct_d);
    max = std::max(max,Spct_d);

    min = std::min(Ppct_d,Vpct_d);
    min = std::min(min,Spct_d);

    if ((max < -1) || (max > 1)) //saturated
        return 1/std::max(std::abs(min),max);
    else
        return 1;       //no saturation
}

//input should be double corresponding to desired thruster force
//2308.60 are the port/stbd thrusters
//using rough approximation: https://github.com/laughlinbarker/openrov_teststand/tree/master/test_stand_data/sample_data_and_output
double OpenROVTeleop::computePctThrustGraupner230860(double &fDes)
{
double pctThrust;

//asuming linear thrust curve w/max fwd thrust 1.5 kg (14.7 N), and 50% rev thrst (7.35 N)

if (fDes > 0)
    pctThrust = fDes/14.7;
if (fDes < 0)
    pctThrust = fDes/7.35;
if (fDes == 0)
    pctThrust = 0;

return pctThrust;
}

//2303.57 is vert thruster, dont have data, but think ~1.3 kg max fwd, approximatly symetrical in ballard pull
//assuming symetical for time being
double OpenROVTeleop::computePctThrustGraupner230357(double &fDes)
{
double pctThrust;

if (fDes > 0)
    pctThrust = fDes/14.7;
if (fDes < 0)
    pctThrust = fDes/7.35;
if (fDes == 0)
    pctThrust = 0;

return pctThrust;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "openrov_teleop");

    OpenROVTeleop rovTeleop;

    ros::spin();
}
