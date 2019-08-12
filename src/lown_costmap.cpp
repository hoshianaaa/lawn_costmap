#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>


class LownCostmap
{
    public:
        LownCostmap();

    private:
        ros::NodeHandle nh_;
        ros::Publisher cloud_pub_;
        ros::Subscriber cloud_sub_;
        ros::Subscriber stop_sub_;
        tf::TransformListener tf_listener_;

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
        void stopCallback(const std_msgs::BoolConstPtr& msg);
        void publishcloud();

        costmap_2d::Costmap2D costmap_;
        int stop_;
};


LownCostmap::LownCostmap()
{

    std::cout << "start LownCostmap" << std::endl;
    costmap_.setDefaultValue(0);
    costmap_.resizeMap(40, 40, 0.1, 0, 0);
   
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/filtered_cloud",1, false);
    cloud_sub_ = nh_.subscribe("velodyne_points", 1, &LownCostmap::cloudCallback, this);
    stop_sub_ = nh_.subscribe("lown_costmap/stop", 1, &LownCostmap::stopCallback, this);

    stop_ = 1;

}

void LownCostmap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
{

    std::cout << "cloud callback" << std::endl;

    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

    double robot_x,robot_y;
    double new_origin_x,new_origin_y;


    if (!stop_){
        pcl::fromROSMsg (*msgs, pcl_cloud);

        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0, 3);
        pass.filter (pcl_cloud);

        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-3, 3);
        pass.filter (pcl_cloud);

        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = "velodyne";

        
        for(int i=0;i<pcl_cloud.width;i++){
            if(i%200==0)
            std::cout << "origin cloud x,y :" << pcl_cloud.points[i].x << ", " << pcl_cloud.points[i].y << std::endl;
        }

        pcl::toROSMsg (pcl_cloud, cloud);

        sensor_msgs::PointCloud2 cloud_odom;

         try
        {
            tf::StampedTransform trans;
            tf_listener_.waitForTransform("odom", "velodyne", ros::Time(0), ros::Duration(0.5));
            tf_listener_.lookupTransform("odom", "velodyne", ros::Time(0), trans);
            pcl_ros::transformPointCloud("odom", trans, cloud, cloud_odom);
        }
        catch(tf::TransformException &e)
        {
                ROS_WARN("%s", e.what());
        }


        pcl::fromROSMsg (cloud_odom, pcl_cloud);

        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("intensity");
        pass.setFilterLimits (40, 60);
        pass.filter (pcl_cloud);

        std::cout << "pcl_cloud size: " << pcl_cloud.width << std::endl;

        unsigned int mx,my;
        double wx,wy;
        for (int i=0;i<pcl_cloud.width;i++){
            wx = pcl_cloud.points[i].x;
            wy = pcl_cloud.points[i].y;
            if(i%30==0)std::cout << "pcl point x,y = " << wx << ", " << wy << std::endl;
            if(costmap_.worldToMap(wx, wy, mx, my))
                costmap_.setCost(mx, my, 1);
        }
    }

    
    try
    {
        tf::StampedTransform trans;
        tf_listener_.waitForTransform("odom", "base_link", 
        ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("odom", "base_link", 
        ros::Time(0), trans);
        robot_x = trans.getOrigin().x();
        robot_y = trans.getOrigin().y();
    }
    catch(tf::TransformException &e)
    {
            ROS_WARN("%s", e.what());
    }

    new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    std::cout << "costmap x,y = " << new_origin_x << ", " << new_origin_y << std::endl;
    costmap_.updateOrigin(new_origin_x, new_origin_y);

    publishcloud();
}


void LownCostmap::stopCallback(const std_msgs::BoolConstPtr& msg)
{
    stop_ = msg->data;
    std::cout << "stop_ = " << stop_ << std::endl;
}


void LownCostmap::publishcloud()
{

    double wx,wy;
    sensor_msgs::PointCloud map_cloud;

    for(int i=0;i<costmap_.getSizeInCellsX();i++){
        for(int j=0;j<costmap_.getSizeInCellsY();j++){
            if(costmap_.getCost(i,j)){
                geometry_msgs::Point32 point;
                costmap_.mapToWorld(i,j,wx,wy);
                point.x = (float)wx;
                point.y = (float)wy;
                map_cloud.points.push_back(point);
            }
        }
    }
    
    std::cout << "publish cloud" << std::endl;
    map_cloud.header.frame_id = "odom";
    cloud_pub_.publish(map_cloud);
}
    
int main(int argc, char **argv)
{
    std::cout << "start node" << std::endl;
    ros::init(argc, argv, "lown_costmap");
    LownCostmap lc;
    ros::Rate r(5);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
