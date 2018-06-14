/// Teleop manager: switches from teleop to autonomous mode using either the joystick or a service

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"

#include <string>
#include <sstream>

using namespace std;

class TeleopManager
{
private:
    int mode; // 0: Teleop; 1: Autonomous;
    bool pressedState; // Press or release logical state
    bool keys_pressed; // last state from joy callback

    std::string controller_type;

    // variables that define joystick keys that combined can switch mode of operation
    int first_key, second_key;
    bool use_two_keys, first_key_isAxis, second_key_isAxis;

    double time_pressed_threshold; // in order to switch, keys have to be pressed simultaneously for <time_pressed_threshold> seconds
    ros::Time start_pressed_time;
    bool reached_time_pressed_threshold;

    // joy_sub gest joystick input in joyCallback
    // cmd_vel_teleop_sub gets velocity commands from the joystick (provided for example from the teleop_twist_joy node) in velTeleopCallback
    // cmd_vel_plan_sub gets velocity commands from autonomous planner (e.g. move_base) in velPlannerCallback
    // cmd_vel_pub publishes velocities from the autonomous planner or the joystick depending on the operation mode
    ros::Subscriber joy_sub, cmd_vel_teleop_sub, cmd_vel_plan_sub;
    ros::Publisher cmd_vel_pub;
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void velTeleOpCallback(const geometry_msgs::Twist::ConstPtr& vel);
    void velPlannerCallback(const geometry_msgs::Twist::ConstPtr& vel);

    // setAutonomousMode receives bool, if true it sets Autonomous Mode (mode=1), otherwise it sets Teleop Mode (mode=0)
    ros::ServiceServer service;
    bool setAutonomousMode(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
public:
    TeleopManager(ros::NodeHandle* nh);
    void run(void);
};

TeleopManager::TeleopManager(ros::NodeHandle* nh){
    cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true); // publishes on /cmd_vel topic
    joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 10, &TeleopManager::joyCallback, this); // subscribes /joy topic
    cmd_vel_teleop_sub = nh->subscribe<geometry_msgs::Twist>("cmd_vel_teleop", 1, &TeleopManager::velTeleOpCallback, this);
    cmd_vel_plan_sub = nh->subscribe<geometry_msgs::Twist>("cmd_vel_planner", 1, &TeleopManager::velPlannerCallback, this);

    service = nh->advertiseService("setAutonomousMode", &TeleopManager::setAutonomousMode, this);

    nh->param("mode", mode, 0); // default mode=0 -> manager node starts in the teleop mode
    if(mode!=0 && mode!=1)
        mode=0;
    //if(!nh->hasParam("mode"))
    nh->setParam("mode", mode);
    if(mode)
        ROS_INFO("Autonomous mode");
    else
        ROS_INFO("Teleop mode");

    nh->param("use_two_keys", use_two_keys, true); // if use_two_keys is true, two keys are needed to switch, otherwise only one;
    //if(!nh->hasParam("use_two_keys"))
    nh->setParam("use_two_keys", use_two_keys);

    nh->param("controller_type", controller_type, std::string("logitech"));
    nh->setParam("controller_type", controller_type);

    if(controller_type.compare("xbox") == 0)
    {
        nh->param("first_key_isAxis", first_key_isAxis, true); // first key can be either an axis or a button
        nh->setParam("first_key_isAxis", first_key_isAxis);
        int first_key_default; // default value for first key is 2 (LT) if axis, and 4 (LB) if button
        if(first_key_isAxis)
            first_key_default=2;
        else
            first_key_default=4;
        nh->param("first_key", first_key, first_key_default);
        if(first_key_isAxis){  // if first key is axis, it can only be 2 or 5 (LT or RT)
            if(first_key!=2 && first_key!=5)
                first_key=2;
        }
        else{ // if first key is button, it can be A, B, X, Y, LB, RB, left stick button or right stick button (not back, start or power)
            if(first_key>=6 && first_key<=8)
                first_key=4;
        }
        nh->setParam("first_key", first_key);
        string message("Press "); // message holds string with information about joystick configuration to switch operation mode
        if(first_key_isAxis)
            message+="Axis ";
        else
            message+="Button ";
        stringstream iss;
        iss<<first_key;
        message+=iss.str();

        if(use_two_keys){
            nh->param("second_key_isAxis", second_key_isAxis, true);
            nh->setParam("second_key_isAxis", second_key_isAxis);
            nh->param("second_key", second_key, 5); // default value for key is 5, RT if axis, and RB if button
            if(second_key_isAxis){  // if first key is axis, it can only be 2 or 5 (LT or RT)
                if(second_key!=2 && second_key!=5)
                        second_key=5;
                if(first_key_isAxis){ // keys have to be different
                    if(first_key==2)
                        second_key=5;
                    else
                        second_key=2;
                }
            }
            else{ // if first key is button, it can be A, B, X, Y, LB, RB, left stick button or right stick button (not back, start or power)
                if(second_key>=6 && second_key<=8)
                    second_key=5;
                if(!first_key_isAxis){ // keys have to be different
                    if(first_key==second_key){
                        for(int i=0;i<11;i++){
                            if(i!=first_key && (i<6 || i>8) ){
                                second_key=i;
                                break;
                            }
                        }
                    }
                }
            }
            nh->setParam("second_key", second_key);
            if(second_key_isAxis)
                message+=" and Axis ";
            else
                message+=" and Button ";
            stringstream iss;
            iss<<second_key;
            message+=iss.str()+" simultaneously";
        }

        nh->param("time_pressed", time_pressed_threshold, 1.0);
        if(time_pressed_threshold<0.0)
            time_pressed_threshold=0.0;
        if(time_pressed_threshold>5.0) // maximum timeout of 5 seconds;
            time_pressed_threshold=5.0;
        nh->setParam("time_pressed", time_pressed_threshold);
        stringstream ss;
        ss<<time_pressed_threshold;
        message+=" for "+ss.str()+" seconds to switch operation mode.";
        ROS_INFO("%s", message.c_str());

        // node requires user to press and then release button/axis in order to switch operation mode
        pressedState=false; // saves trigger state;
        keys_pressed=false; // current joystick triggers state from joy callback;

        start_pressed_time=ros::Time::now();
        reached_time_pressed_threshold=false;
    }
    else if(controller_type.compare("logitech") == 0)
    {
        nh->param("first_key_isAxis", first_key_isAxis, false); // first key can be either an axis or a button
        //nh->setParam("first_key_isAxis", first_key_isAxis);
        nh->param("first_key", first_key, 6);
        //nh->setParam("first_key", first_key);
        string message("Press "); // message holds string with information about joystick configuration to switch operation mode
        if(first_key_isAxis)
            message+="Axis ";
        else
            message+="Button ";
        stringstream iss;
        iss<<(first_key+1);
        message+=iss.str();

        if(use_two_keys){
            nh->param("second_key_isAxis", second_key_isAxis, false);
            //nh->setParam("second_key_isAxis", second_key_isAxis);
            nh->param("second_key", second_key, 7); // default value for key is 5, RT if axis, and RB if button
            //nh->setParam("second_key", second_key);
            if(second_key_isAxis)
                message+=" and Axis ";
            else
                message+=" and Button ";
            stringstream iss;
            iss<<(second_key+1);
            message+=iss.str()+" simultaneously";
        }

        nh->param("time_pressed", time_pressed_threshold, 1.0);
        if(time_pressed_threshold<0.0)
            time_pressed_threshold=0.0;
        if(time_pressed_threshold>5.0) // maximum timeout of 5 seconds;
            time_pressed_threshold=5.0;
        //nh->setParam("time_pressed", time_pressed_threshold);
        stringstream ss;
        ss<<time_pressed_threshold;
        message+=" for "+ss.str()+" second to switch operation mode.";
        ROS_INFO("%s", message.c_str());

        // node requires user to press and then release button/axis in order to switch operation mode
        pressedState=false; // saves trigger state;
        keys_pressed=false; // current joystick triggers state from joy callback;

        start_pressed_time=ros::Time::now();
        reached_time_pressed_threshold=false;
    }
}

bool TeleopManager::setAutonomousMode(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    if(req.data){
        mode=1;
        res.success=true;
        res.message="Manager in Autonomous Mode";
        ROS_INFO("Autonomous mode is now active.");
    }
    else{
        mode=0;
        res.success=true;
        res.message="Manager in Teleop Mode";
        ROS_INFO("Teleop mode is now active.");
    }
    return true;
}

void TeleopManager::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    bool first_key_pressed;
    float axis_threshold=0.5;
    // code for setting triggers state
    if(first_key_isAxis){
        if(!keys_pressed){ // threshold is different depending on last state - hysteresis
            if(joy->axes[first_key]<-axis_threshold)
                first_key_pressed=true;
            else
                first_key_pressed=false;
        }
        else{
            if(joy->axes[first_key]>axis_threshold)
                first_key_pressed=false;
            else
                first_key_pressed=true;
        }
    }
    else{
        if(joy->buttons[first_key]==0)
            first_key_pressed=false;
        else
            first_key_pressed=true;
    }
    if(use_two_keys){
        bool second_key_pressed;
        if(second_key_isAxis){
            if(!keys_pressed){ // threshold is different depending on last state - hysteresis
                if(joy->axes[second_key]<-axis_threshold)
                    second_key_pressed=true;
                else
                    second_key_pressed=false;
            }
            else{
                if(joy->axes[second_key]>axis_threshold)
                    second_key_pressed=false;
                else
                    second_key_pressed=true;
            }
        }
        else{
            if(joy->buttons[second_key]==0)
                second_key_pressed=false;
            else
                second_key_pressed=true;
        }
        keys_pressed=first_key_pressed && second_key_pressed; // overall trigger state is AND of each key trigger states
    }
    else{
        keys_pressed=first_key_pressed;
    }
}

void TeleopManager::velTeleOpCallback(const geometry_msgs::Twist::ConstPtr& vel){
    if(!mode) // publishes received velocity command from joystick if mode==0 (teleop mode)
        cmd_vel_pub.publish(vel);
}

void TeleopManager::velPlannerCallback(const geometry_msgs::Twist::ConstPtr& vel){
    if(mode)  // publishes received velocity command from planner if mode==1 (autonomous mode)
        cmd_vel_pub.publish(vel);
}

void TeleopManager::run(void){
    // state machine to detect joystick press and release sequence using triggers state
    if(!pressedState && keys_pressed){ // previously NOT pressed and detects both triggers are currently pressed
        pressedState=true; // changes state to pressed
        start_pressed_time=ros::Time::now(); // start counting time
    }
    else if(pressedState && !keys_pressed){ // previously pressed and detects both triggers are currently NOT pressed
        pressedState=false; // changes state to NOT pressed
        reached_time_pressed_threshold=false;
    }
    else if(pressedState && keys_pressed && !reached_time_pressed_threshold){ // previously and currently pressed, and timeout not reached before
        ros::Duration duration=ros::Time::now()-start_pressed_time;
        if(duration.toSec() >= time_pressed_threshold){ // Pressed triggers time reached duration needed to switch mode
            mode=1-mode; //switches operation mode

            // Print string with current operation mode
            std::string currMode=mode?"Planner":"Teleop";
            currMode="Switched to "+ currMode+" mode";
            ROS_INFO("%s", currMode.c_str());

            // sends zero velocity command whenever switching operation mode
            geometry_msgs::Twist vel;
            vel.linear.x=0;
            vel.linear.y=0;
            vel.linear.z=0;
            vel.angular.x=0;
            vel.angular.y=0;
            vel.angular.z=0;
            cmd_vel_pub.publish(vel);

            reached_time_pressed_threshold=true; // change state to wait for keys to be released
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pepper_teleop_manager");
    ros::NodeHandle nh("~");
    TeleopManager manager(&nh);
    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
        manager.run();
        loop_rate.sleep();
    }
    ros::spin();
}
