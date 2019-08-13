#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>



class LawnCostmap
{
    public:
        LawnCostmap();

    private:
        ros::NodeHandle nh_;
        ros::Publisher cloud_pub_;
        ros::Subscriber cloud_sub_;
        ros::Subscriber activate_sub_;
        tf::TransformListener tf_listener_;

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
        void activateCallback(const std_msgs::BoolConstPtr& msg);
        void publishcloud();

        costmap_2d::Costmap2D costmap_;
        int activate_;
};


LawnCostmap::LawnCostmap()
{

    std::cout << "start LawnCostmap" << std::endl;
    costmap_.setDefaultValue(0);
    costmap_.resizeMap(40, 40, 0.1, 0, 0);
   
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud",1, false);
    cloud_sub_ = nh_.subscribe("velodyne_points", 1, &LawnCostmap::cloudCallback, this);
    activate_sub_ = nh_.subscribe("lawn_costmap/activate", 1, &LawnCostmap::activateCallback, this);

    //activate_ = 0;
    activate_ = 1;

}

void LawnCostmap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
{

    std::cout << "cloud callback" << std::endl;

    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::PointCloud<pcl::PointXYZI> remove_pcl_cloud;

    double robot_x,robot_y;
    double new_origin_x,new_origin_y;

    if (activate_){
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

        //高さのあるものを除く
        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-0.3, 0.3);
        pass.filter (remove_pcl_cloud);

        //-0.5~-0.45くらいが地面
        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-0.6, -0.3);
        pass.filter (pcl_cloud);

        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = "velodyne";

        pcl::toROSMsg (pcl_cloud, cloud);

        sensor_msgs::PointCloud2 cloud_odom;

        try
        {
            tf::StampedTransform trans;
            tf_listener_.waitForTransform("odom", "velodyne", ros::Time(0), ros::Duration(0.5));
            tf_listener_.lookupTransform("odom", "velodyne", ros::Time(0), trans);
            pcl_ros::transformPointCloud("odom", trans, cloud, cloud_odom);
            pcl_ros::transformPointCloud("odom", trans, cloud, cloud_odom);
        }
        catch(tf::TransformException &e)
        {
            ROS_WARN("%s", e.what());
        }


        pcl::fromROSMsg (cloud_odom, pcl_cloud);

        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("intensity");
        pass.setFilterLimits (45, 60);
        pass.filter (pcl_cloud);

        /////  filter /////
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud (pcl_cloud.makeShared());
        sor.setMeanK (5);
        sor.setStddevMulThresh (0.1);
        sor.filter (pcl_cloud);
        

        std::cout << "pcl_cloud size: " << pcl_cloud.width << std::endl;

        unsigned int mx,my;
        double wx,wy;
        for (int i=0;i<pcl_cloud.width;i++){
            wx = pcl_cloud.points[i].x;
            wy = pcl_cloud.points[i].y;
            if(costmap_.worldToMap(wx, wy, mx, my)){
                costmap_.setCost(mx, my, 1);
            }
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
    costmap_.updateOrigin(new_origin_x, new_origin_y);

    publishcloud();
}


void LawnCostmap::activateCallback(const std_msgs::BoolConstPtr& msg)
{
    activate_ = msg->data;
    std::cout << " activate_ = " << activate_ << std::endl;
}


void LawnCostmap::publishcloud()
{

    double wx,wy;
    sensor_msgs::PointCloud map_cloud_odom;
    sensor_msgs::PointCloud2 map_cloud2_odom;
    sensor_msgs::PointCloud2 map_cloud2_velodyne;

    
    for(int i=0;i<costmap_.getSizeInCellsX();i++){
        for(int j=0;j<costmap_.getSizeInCellsY();j++){
            if(costmap_.getCost(i,j)){
                geometry_msgs::Point32 point;
                costmap_.mapToWorld(i,j,wx,wy);
                point.x = (float)wx;
                point.y = (float)wy;
                map_cloud_odom.points.push_back(point);
            }
        }
    }
    
    std::cout << "publish cloud" << std::endl;
    map_cloud_odom.header.frame_id = "odom";

    sensor_msgs::convertPointCloudToPointCloud2 (map_cloud_odom, map_cloud2_odom);

    try
    {
        tf::StampedTransform trans;
        tf_listener_.waitForTransform("odom", "velodyne", ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("velodyne", "odom", ros::Time(0), trans);
        pcl_ros::transformPointCloud("velodyne", trans, map_cloud2_odom, map_cloud2_velodyne);
    }
    catch(tf::TransformException &e)
    {
        ROS_WARN("%s", e.what());
    }

    cloud_pub_.publish(map_cloud2_velodyne);
}

int main(int argc, char **argv)
{
    std::cout << "start node" << std::endl;
    ros::init(argc, argv, "lawn_costmap");
    LawnCostmap lc;
    ros::Rate r(5);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
