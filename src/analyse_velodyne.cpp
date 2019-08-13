#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

class AnalyseVelodyne
{
public:
    AnalyseVelodyne()
    {
        cloud_sub_ = nh_.subscribe("velodyne_points", 1, &AnalyseVelodyne::cloudCallback, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("analyse_velodyne/filtered_cloud", 1, false);
        cloud_pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("analyse_velodyne/filtered_cloud2", 1, false);
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher cloud_pub2_;
    
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
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
        
        //-0.5~-0.45くらいが地面
        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-0.6, -0.3);
        pass.filter (pcl_cloud);

        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("intensity");
        pass.setFilterLimits (45, 60);
        pass.filter (pcl_cloud);

        sensor_msgs::PointCloud2 cloud;
        pcl::toROSMsg (pcl_cloud, cloud);
        cloud.header.frame_id = "velodyne";
        cloud_pub_.publish (cloud);

        /////  filter /////
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud (pcl_cloud.makeShared());
        sor.setMeanK (5);
        sor.setStddevMulThresh (0.1);
        sor.filter (pcl_cloud);
 
        sensor_msgs::PointCloud2 cloud2;
        pcl::toROSMsg (pcl_cloud, cloud2);
        cloud2.header.frame_id = "velodyne";
        cloud_pub2_.publish (cloud2);

    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "analyse_vel");
    AnalyseVelodyne av;
    ros::Rate r(5);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

