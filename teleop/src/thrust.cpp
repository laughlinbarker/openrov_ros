/* Node to calculate required ESC command to achive desired ROV velocities
 * Laughlin Barker | 2016
 * laughlin@jhu.edu
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/UInt16.h>
#include <eigen/Eigen/Core>
#include <eigen/unsupported/Eigen/MatrixFunctions>

class OpenROVTeleop
{
public:
    OpenROVTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh;

    int x, z, yaw; //joy msg indicies for respective movements
    double x_gain, z_gain, yaw_gain;    //gain for respective movements

    //published topics - future improvement: custom OpenROV msg that contains all
    ros::Publisher motorPub;
    ros::Publisher lightPub;
    ros::Publisher laserPub;
    ros::Publisher camTiltPub;

    ros::Subscriber statusMsgSub;
    ros::Subscriber cameraTiltSub;
    ros::Subscriber battCurrentSub;
    ros::Subscriber cpuUsageSub;
    ros::Subscriber hotelCurrentSub;
    ros::Subscriber battVoltageSub;
    ros::Subscriber laserStatusSub;
    ros::Subscriber lightLevelSub;
    ros::Subscriber navDadaSub;
    ros::Subscriber temperatureSub;
    ros::Subscriber pressureSub;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "thrust_comp");

    ros::NodeHandle nh;

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback, this);
    ros::Publisher pub = nh.advertise<std_msgs::Uint16>("openrov/motor_command",1);
}

//sub scribe to desired Twist msg

//calculate required forces/torques in ROV body frame
//Fx =
//Fz =
//Mz =

//calculate required motor forces
//Tau = inv(A)*F

//deal with saturation

//calculate required ESC command to achieve required motor force

//publish ESC command to /openrov/motor_cmd topic

//joyCallback
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

}
