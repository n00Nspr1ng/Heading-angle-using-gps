#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
// #include <tf/tf.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <robot_localization/navsat_conversions.h>

//여기
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h> // quaternion 변환 함수 들어있음
#include <math.h>

#define Pi 3.141592
#define InitialPointRaw -5.0 // rviz에 displacement_raw 원점 설정 : (-5.0, -5.0)에 화살표 그려짐
#define InitialPointFiltered -2.0 // rviz에 displacement_filtered 원점 x좌표 설정 : (-2.0, InitialPointRaw)에 화살표 그려짐
//까지

using namespace std;

class headingAngle
{
public:
    headingAngle(ros::NodeHandle &nh):
    
    //Subscribe to gpu topic
    gpu_sub(nh.subscribe("/fix",100,&headingAngle::fix_CB,this)),
    
    //Publish heading angle
    heading_pub(nh.advertise<geometry_msgs::PointStamped>("/heading_angle",10)),
    gps_pub(nh.advertise<sensor_msgs::NavSatFix>("/gps_point",100)),

    //여기
    // 필터링 안 된/된 heading angle Topic
    heading_raw_pub(nh.advertise<std_msgs::Float32>("/heading_raw",10)),
    heading_filtered_pub(nh.advertise<std_msgs::Float32>("/heading_filtered",10)),
    // rviz에 필터링 안 된/된 방향 확인하기 위한 Topic들
    displacement_raw_pub(nh.advertise<nav_msgs::Odometry>("/displacement_raw",10)),
    displacement_filtered_pub(nh.advertise<nav_msgs::Odometry>("/displacement_filtered",10)),
    //까지

    loop_rate(20)
    {
        init();
    }

    void init()
    {
        initGPS();
        loadParam();

        //여기
        initDisplayData();
        //까지
    }

    void spin()
    {
        while(ros::ok())
        {
            ros::spinOnce();
            heading_pub.publish(heading);
            gps_pub.publish(gps_point);
            
            //여기
            heading_raw_pub.publish(heading_raw);
            displacement_raw_pub.publish(displacement_raw);

            heading_filtered_pub.publish(heading_filtered);
            displacement_filtered_pub.publish(displacement_filtered);
            //까지

            loop_rate.sleep();
        }
    }

    void initGPS(void)
    {
        heading.header.frame_id = "odom";
        //heading.header.child_frame_id = "base_footprint";
        heading.header.seq = 0.0;
        heading.point.x = 0.0;
        heading.point.y = 0.0;
        heading.point.z = 0.0;

        gps_point.header.frame_id = "odom";
        gps_point.header.seq = 0.0;
        gps_point.point.x = 0.0;
        gps_point.point.y = 0.0;
        gps_point.point.z = 0.0;

        ROS_ERROR_STREAM("initGPS done");

    }

    void loadParam()
    {
        nh.getParam("threshold_distance", min_dis);
        ROS_ERROR_STREAM("minimum distance =" << min_dis);

    }

    //여기
    void initDisplayData()
    {
        heading_raw.data = 0.0;

        displacement_raw.header.frame_id = "odom";
        displacement_raw.child_frame_id = "displacement_raw_footprint";
        displacement_raw.pose.pose.position.x = InitialPointRaw;
        displacement_raw.pose.pose.position.y = InitialPointRaw;
        displacement_raw.pose.pose.position.z = 0.0;
        displacement_raw.pose.pose.orientation.x = 0.0;
        displacement_raw.pose.pose.orientation.y = 0.0;
        displacement_raw.pose.pose.orientation.z = 0.0;
        displacement_raw.pose.pose.orientation.w = 0.0;

        heading_raw_quaternion.setRPY(0, 0, 0);

        heading_filtered.data = 0.0;

        displacement_filtered.header.frame_id = "odom";
        displacement_filtered.child_frame_id = "displacement_filtered_footprint";
        displacement_filtered.pose.pose.position.x = InitialPointFiltered;
        displacement_filtered.pose.pose.position.y = InitialPointRaw;
        displacement_filtered.pose.pose.position.z = 0.0;
        displacement_filtered.pose.pose.orientation.x = 0.0;
        displacement_filtered.pose.pose.orientation.y = 0.0;
        displacement_filtered.pose.pose.orientation.z = 0.0;
        displacement_filtered.pose.pose.orientation.w = 0.0;

        heading_filtered_quaternion.setRPY(0, 0, 0);
    }

    void calculate_displacement_raw(double dx, double dy)
    {
        // 필터링 안 된 heading angle 계산
        heading_raw.data = atan(dy / dx);
        if(dx < 0 && dy > 0)
            heading_raw.data = heading_raw.data + Pi;
        else if(dx < 0 && dy < 0)
            heading_raw.data = heading_raw.data - Pi;

        // heading angle로 quaternion 변환        
        heading_raw_quaternion.setRPY( 0, 0, heading_raw.data );

        displacement_raw.pose.pose.position.x = InitialPointRaw + dx;
        displacement_raw.pose.pose.position.y = InitialPointRaw + dy;

        displacement_raw.pose.pose.orientation.x = heading_raw_quaternion.getX();
        displacement_raw.pose.pose.orientation.y = heading_raw_quaternion.getY();
        displacement_raw.pose.pose.orientation.z = heading_raw_quaternion.getZ();
        displacement_raw.pose.pose.orientation.w = heading_raw_quaternion.getW();
    }

    void calculate_displacement_filtered(double dx, double dy)
    {
        // 필터링 된 heading angle 계산
        heading_filtered.data = atan(dy / dx);
        if(dx < 0 && dy > 0)
            heading_filtered.data = heading_filtered.data + Pi;
        else if(dx < 0 && dy < 0)
            heading_filtered.data = heading_filtered.data - Pi;

        // heading angle로 quaternion 변환        
        heading_filtered_quaternion.setRPY( 0, 0, heading_filtered.data );

        displacement_filtered.pose.pose.position.x = InitialPointFiltered + dx;
        displacement_filtered.pose.pose.position.y = InitialPointRaw + dy;

        displacement_filtered.pose.pose.orientation.x = heading_filtered_quaternion.getX();
        displacement_filtered.pose.pose.orientation.y = heading_filtered_quaternion.getY();
        displacement_filtered.pose.pose.orientation.z = heading_filtered_quaternion.getZ();
        displacement_filtered.pose.pose.orientation.w = heading_filtered_quaternion.getW();
    }
    //까지

    void calculate_heading()
    {
        double dx = gps_point.point.x - prev_x;
        double dy = gps_point.point.y - prev_y;
        double dt = gps_point.header.stamp.toSec() - prev_time;
        double theta = 180/Pi*atan2(dx, (-dy));
        //double vel = sqrt((dx/dt)*(dx/dt) + (dy/dt)*(dy/dt));
        //ROS_ERROR_STREAM("velocity " << vel);

        if (dt != 0)
        {
            if (dx > min_dis || dy > min_dis)
            {
                ROS_ERROR_STREAM("calculate heading " << gps_point.point.x << " " << gps_point.point.y);
                
                if (check_rad(theta))
                {
                    heading.header.stamp = ros::Time::now();
                    heading.point.x = theta;
                    heading.point.y = dt;    

                    //여기
                    calculate_displacement_filtered(dx, dy);
                    //까지
                }
            }
        }

        //여기
        calculate_displacement_raw(dx, dy);
        //까지

        prev_x = gps_point.point.x;
        prev_y = gps_point.point.y;
        prev_time = gps_point.header.stamp.toSec();
    }

    bool check_rad(double theta)
    {
        ROS_ERROR_STREAM("prev " << heading.point.x << " now " << theta);
        
        double diff = abs(heading.point.x - theta);
        if (diff < max_angle || diff > 360 - max_angle)
        {//going forward
            ROS_ERROR_STREAM("true");
            return true;
        }
        else if (abs(diff - 180) < max_angle)
        {//going backward
            ROS_ERROR_STREAM("true with backward");
            forward_check = false;
            return true;
        }
        else
        {
            ROS_ERROR_STREAM("false");
            return false;
        }

        // ROS_ERROR_STREAM("prev " << heading.point.x << " now " << theta);
        // double diff = abs(heading.point.x - theta);
        // if (diff < max_angle || abs(180 - diff) < max_angle || abs(360 - diff) < max_angle)
        // {
        //     ROS_ERROR_STREAM("true");
        //     return true;
        // }
        // else 
        // {
        //     ROS_ERROR_STREAM("false");
        //     return false;
        // }
    }


    void fix_CB(const sensor_msgs::NavSatFix &navsat_msg)
    {
        double utm_x = 0, utm_y = 0;
        std::string utm_zone;
        //convert lat/long to utm
        RobotLocalization::NavsatConversions::LLtoUTM(navsat_msg.latitude, navsat_msg.longitude, utm_x, utm_y, utm_zone);

        if(!navSat_msg_received){
            if(gps_origin_x_ == 0 && gps_origin_y_ == 0)
            {
            gps_origin_x_ = utm_x;
            gps_origin_y_ = utm_y;
            }
            odom_origin_x_ = utm_x;
            odom_origin_y_ = utm_y;
            navSat_msg_received = true;
            prev_time = ros::Time::now().toSec();
        }
        gps_point.header.stamp = ros::Time::now();
        gps_point.point.x = utm_x - odom_origin_x_;
        gps_point.point.y = -(utm_y - odom_origin_y_);
        gps_point.point.z = 0;
        calculate_heading();
    }


private:
    /*Subscriber */
    ros::Subscriber gpu_sub;
    
    /*Publisher */
    ros::Publisher heading_pub;
    ros::Publisher gps_pub;

    /*Transform Broadcaster */

    /*Parameter*/
    bool navSat_msg_received = false;
    double gps_origin_x_ = 0;
    double gps_origin_y_ = 0;
    double odom_origin_x_ = 0;
    double odom_origin_y_ = 0;

    double prev_x = 0;
    double prev_y = 0;
    double prev_time = 0;

    double min_dis = 0.25;
    //double min_vel = 0.0;
    double max_angle = 40.0;
    bool forward_check = true;


    /*ROS Variable*/
    ros::NodeHandle nh;
    ros::Rate loop_rate;  

    /*Message Types*/
    geometry_msgs::PointStamped heading;
    geometry_msgs::PointStamped gps_point;

    //여기
    ros::Publisher heading_raw_pub;
    ros::Publisher heading_filtered_pub;

    ros::Publisher displacement_raw_pub;
    ros::Publisher displacement_filtered_pub;

    std_msgs::Float32 heading_raw; // 필터링 안 된 heading
    nav_msgs::Odometry displacement_raw; // 필터링 안 된 변위 rviz 표시용
    tf2::Quaternion heading_raw_quaternion; // 필터링 안 된 변위 quaternion 변환용

    std_msgs::Float32 heading_filtered; // 필터링 된 heading
    nav_msgs::Odometry displacement_filtered; // 필터링 된 변위 rviz 표시용
    tf2::Quaternion heading_filtered_quaternion; // 필터링 된 변위 quaternion 변환용
    //까지
		
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heading_angle");
    ros::NodeHandle nh;
    headingAngle core(nh);
    core.spin();
    return 0;
}
