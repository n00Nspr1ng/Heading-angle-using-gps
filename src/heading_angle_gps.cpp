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

    loop_rate(20)
    {
        init();
    }

    void init()
    {
        initGPS();
        loadParam();
    }

    void spin()
    {
        while(ros::ok())
        {
            ros::spinOnce();
            heading_pub.publish(heading);
            gps_pub.publish(gps_point);
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

    void calculate_heading()
    {
        double dx = gps_point.point.x - prev_x;
        double dy = gps_point.point.y - prev_y;
        double dt = gps_point.header.stamp.toSec() - prev_time;
        double theta = 180/3.14*atan2(dx, (-dy));
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
                }

            }
            
        }

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
		
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heading_angle");
    ros::NodeHandle nh;
    headingAngle core(nh);
    core.spin();
    return 0;
}
