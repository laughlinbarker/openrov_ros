/* Laughlin Barker | 2016
 * laughlin@jhu.edu
 * Written for EN 530.707 | Robot Systems Programming | Johns Hopkins University
 */

#include <ros/ros.h>

#include <math.h>

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
    ros::Timer timer;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    double limitThrusterSaturation(double &Ppct_d, double &Vpct_d, double &Spct_d);
    double computePctThrustGraupner230860(double &fDes);
    double computePctThrustGraupner230357(double &fDes);

    //joy node, when publishing gamepad topics sends at 100-200 Hz, which is way to fast for OpenROV to handle,
    //so we create timer and send OpenROV cmds at regular interval
    void timerCallback(const ros::TimerEvent& event);


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
    z_controllerAxis(4),       //right stick up/down
    yaw_controllerAxis(0),      //left stick left/right
    lightsAdj(6),   // cross key left/right
    laserToggle(10),    //button stick right
    camTilt(7),      // cross key up/down
    x_gain(3),
    z_gain(3),
    yaw_gain(0.3)
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

    //std::cout << "Requested Mz: " << mz_d << std::endl;

    // thruster allocation matrix
    A << 1, 0, 1,
         0, 1, 0,
         -d, 0, d;      //full rank

    // ROV body frame forces/torques [fx, fz, mz]' - marine body conventions: x: forward, y: stdb, z: down
    Eigen::Vector3d F(fx_d,fz_d,mz_d);

    //std::cout << "F: " << F << std::endl;

    // solve for thruster force vector [T_port, T_vert, T_stbd]'
    Eigen::Vector3d T =  A.inverse() * F;  // A.inv safe because A full rank (by inspection, above) <--> invertiable


    //std::cout << "Requested motor torques: " << T << std::endl;

    //calculate desired percentage thrust from each thruster
    double Ppct_d, Vpct_d, Spct_d;
    Ppct_d = computePctThrustGraupner230860(T(0));
    Vpct_d = computePctThrustGraupner230357(T(1));
    Spct_d = computePctThrustGraupner230860(T(2));

    //std::cout << "Percent Thrust requested: [Port, Vert, Stbd]: [" << Ppct_d << "," << Vpct_d << "," << Spct_d << "]" << std::endl;

    //now we deal with thruster saturation - ROV pilots often prefer prioritizing heading authority over
    //but for now lets just scale everything to bring within saturation limits
    double scaleFactor = limitThrusterSaturation(Ppct_d, Vpct_d, Spct_d);

    //std::cout << "Scale Factor: " << scaleFactor << std::endl;

    //scale thrust appropriately, map to [1000, 2000] ms range for servos
    int Pms, Vms, Sms;
    Pms = 1500 + round((Ppct_d * scaleFactor) * 500);
    Vms = 1500 + round(( Vpct_d * scaleFactor) * 500);
    Sms = 1500 + round((Spct_d * scaleFactor) * 500);

    //std::cout << "ESC vals: " << Pms << "," << Vms << "," << Sms << std::endl;

    motor_cmds.motors[0] = Pms;
    motor_cmds.motors[1] = Vms;
    motor_cmds.motors[2] = Pms;

    //motorPub.publish(motor_cmds);

}

void OpenROVTeleop::timerCallback(const ros::TimerEvent& event)
{
    motorPub.publish(motor_cmds);
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

    //std::cout << "max/min: " << max << "," << min << std::endl;

    if ((min < -1) || (max > 1)) //saturated
        return 1/std::max(std::abs(min),max); //this is effectively a scale factor s.t. Fscaled = Fsaturated * scale_factor
    else
        return 1;       //no saturation
}

//input should be double corresponding to desired thruster force
//2308.60 are the port/stbd thrusters
//using rough approximation: https://github.com/laughlinbarker/openrov_teststand/tree/master/test_stand_data/sample_data_and_output
double OpenROVTeleop::computePctThrustGraupner230860(double &fDes)
{

    //std::cout << "Fdes 2308: " << fDes << std::endl;
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
    //std::cout << "Fdes 2303: " << fDes << std::endl;
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

    //
    rovTeleop.timer = rovTeleop.nh.createTimer(ros::Duration(0.2), &OpenROVTeleop::timerCallback, &rovTeleop);

    ros::spin();
}
