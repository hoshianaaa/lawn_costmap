#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>

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
        tf::TransformListener tf_listener_;

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
        void publishcloud();

        costmap_2d::Costmap2D costmap_;
};


LownCostmap::LownCostmap()
{

    std::cout << "start LownCostmap" << std::endl;
    costmap_.setDefaultValue(0);
    costmap_.resizeMap(40, 40, 0.1, 0, 0);
   
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/filtered_cloud",1, false);
    cloud_sub_ = nh_.subscribe("velodyne_points", 1, &LownCostmap::cloudCallback, this);

}

void LownCostmap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
{

    std::cout << "cloud callback" << std::endl;

    double robot_x,robot_y;
    double new_origin_x,new_origin_y;

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

    tf::StampedTransform trans;
    sensor_msgs::PointCloud2 trans_cloud2;
    pcl_ros::transformPointCloud("odom", trans, *msgs, trans_cloud2);

    new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);

    publishcloud();
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