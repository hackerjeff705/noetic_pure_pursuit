#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <bits/stdc++.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <math.h>       /* atan, sin */

#define _USE_MATH_DEFINES

using namespace std;

// GLOBAL VARIABLES
int freqs = 10;
int idx = 0;
float xc = 0.;
float yc = 0.;
float vel = 0.;
float yaw = 0.;
float v_prev_error = 0.;
vector<vector<float>> waypoints;

// CAR VARIABLES
float LOOKAHEAD = 0.1;
float WB = 0.04; // wheelbase

// PORGRAM VARIABLES
bool pure_pursuit_flag = true;

// Initialize Twist message
geometry_msgs::Twist msg;



// ------ READ CSV ------ //
vector<vector<float>> read_points(string fname) {
    vector<vector<float>> array;
    vector<float> row;
    string line, word;
    float val;
 
    fstream file (fname, ios::in);
    while(getline(file, line))
    {
        row.clear();
        stringstream str(line);
 
        while(getline(str, word, ',')) {
            // Conver string to float
            val = stof(word);
            row.push_back(val);
        }
        array.push_back(row);
    }
    return array;
}

void read_file() 
{
    string fname = "/home/{REPLACE_WITH_FILE_PATH}/pure_pursuit/waypoints/wp_file.csv";
 
    fstream file (fname, ios::in);

    if(file.is_open())
    {
        waypoints = read_points(fname);
    }
    else
        ROS_INFO_STREAM("Could not open the file");
}

// ----- ODOMETRY ------ //
float norm(vector<float> vect)
{
    float sum = 0.;
    for (int i = 0; i < vect.size(); ++i) {
        sum += pow(vect[i], 2.);
    }
    return sqrt(sum);
}

/**
 * Callback function executes when new topic data comes.
 * Collects latest data for postion, orientation and velocity
 */
void pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Position
    xc = msg->pose.pose.position.x;
    yc = msg->pose.pose.position.y;
    float zc = msg->pose.pose.position.z;

    // Orientation
    float qx = msg->pose.pose.orientation.x;
    float qy = msg->pose.pose.orientation.y;
    float qz = msg->pose.pose.orientation.z;
    float qw = msg->pose.pose.orientation.w;

    // Linear velocity
    float linear_vx = msg->twist.twist.linear.x;
    float linear_vy = msg->twist.twist.linear.y;
    float linear_vz = msg->twist.twist.linear.z;

    vector<float> vect_vel = {linear_vx, linear_vy, linear_vz};
    float linear_vel = norm(vect_vel);

    // Angular velocity
    float angular_vel = msg->twist.twist.angular.z;

    // Euler from Quaternion
    tf::Quaternion quat(qx, qy, qz, qw);
    tf::Matrix3x3 mat(quat);

    double curr_roll, curr_pitch, curr_yaw;
    mat.getEulerYPR(curr_yaw, curr_pitch, curr_roll);

    // Assign to global variables
    vel = linear_vel;
    yaw = curr_yaw;

    // TESTING: Print values 
    // ROS_INFO("GLOBAL Position [%f, %f, %f], vel [%f], yaw [%f]", xc, yc, zc, vel, yaw);
    // ROS_INFO("Roll, Pitch, Yaw = [%f, %f, %f]", roll, pitch, yaw);
    // ROS_INFO("Seq -> [%d], Vel (linear x, norm, angular) -> [%f, %f, %f]", msg->header.seq, linear_vx, linear_vel, angular_vel);
    // ROS_INFO("Vel (linear x, norm) -> [%f, %f]", linear_vx, linear_vel);
    // ROS_INFO("Position (xc, yc, zc) -> [%f, %f, %f]", xc, yc, zc);
    // ROS_INFO("Orientation (qx, qy, qz, qw) -> [%f, %f, %f, %f]", qx, qy, qz, qw);
    // ROS_INFO("\n");
}

// ----- ARRAY MANIOULATION ------ //
float find_distance(float x1, float y1)
{
    float P = 2.0; // power of 2
    float distance = sqrt(pow(x1 - xc, P) + pow(y1 - yc, P));
    return distance;
}

float find_distance_index_based(int idx)
{
    float x1 = waypoints[idx][0];
    float y1 = waypoints[idx][1];
    return find_distance(x1, y1);
}

int find_nearest_waypoint()
{
    int nearest_idx = 0;
    float smallest_dist = 0.;
    float P = 2.;
    for (int i = 0; i < waypoints.size(); i++)
    {
        float wpx = waypoints[i][0];
        float wpy = waypoints[i][1];
        float idx_dist = pow(xc - wpx, P) + pow(yc - wpy, P);

        if (i == 0) {
            smallest_dist = idx_dist;
            nearest_idx = i;
        }
        if (idx_dist < smallest_dist ) {
            smallest_dist = idx_dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

int idx_close_to_lookahead(int idx)
{
    while (find_distance_index_based(idx) < LOOKAHEAD) {
        idx += 1;
        if (idx == waypoints.size()) { break; }
    }
    return idx - 1;
}

void PurePursuit()
{
    // Get the closest waypoint
    int nearest_idx = find_nearest_waypoint();
    idx = idx_close_to_lookahead(nearest_idx);
    float target_x = waypoints[idx][0];
    float target_y = waypoints[idx][1];

    // Velocity PID controller
    float kp = 1.;
    float kd = 0.;
    float ki = 0.;

    float dt = 1. / freqs;
    float v_desired = waypoints[nearest_idx][3];
    float v_error = v_desired - vel;

    float P_vel = kp * v_error;
    float I_vel = v_error * dt;
    float D_vel = kd * (v_error - v_prev_error) / dt;

    float velocity = P_vel + I_vel + D_vel;
    v_prev_error = v_error;

    // Pure Pursuit controller
    float x_delta = target_x - xc;
    float y_delta = target_y - yc;
    float alpha = atan(y_delta / x_delta) - yaw;

    if (alpha > M_PI_2) { alpha -= M_PI; }
    if (alpha < -M_PI_2) { alpha += M_PI; }

    // Set lookahead distance depending on the speed
    float lookahead = find_distance(target_x, target_y);
    float steering_angle = atan((2. * WB * sin(alpha)) / lookahead);

    // Set max wheel turning angle
    if (steering_angle > 0.5) {
        steering_angle = 0.5;
    } else if (steering_angle < -0.5) {
        steering_angle = -0.5;
    }

    // Publish the message
    msg.linear.x = velocity;
    msg.angular.z = steering_angle;
}


// ------ MAIN FUNCTION ------ //
int main(int argc, char **argv)
{
    // Load waypoints
    read_file();

    // Initialize node
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;

    // Initialize Subscriber
    ros::Subscriber controller_sub = nh.subscribe("odom", 1000, pose_callback);

    // Initizlize Publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Initialize rate
    ros::Rate rate(freqs);
    ros::spinOnce();

    // MOVE ROBOT
    while(ros::ok()) 
    {
        // Send a message to rosout with the details.
        ROS_INFO_STREAM("index = " << waypoints.size() << " idx = " << idx << " current pose [" << xc << "," << yc << "] [vel, yaw] = [" << vel << "," << yaw << "]");
        if (idx < waypoints.size() - 1) {
            PurePursuit();
            pub.publish(msg);

            // Wait until it's time for another iteration.
            ros::spinOnce();
            rate.sleep();
        } else {
            // Publish the message
            msg.linear.x = 0.;
            msg.angular.z = 0.;
            pub.publish(msg);
            break;
        }
    }

    ROS_INFO_STREAM("DESTENATION REACHED!!!");
    return 0;
}
