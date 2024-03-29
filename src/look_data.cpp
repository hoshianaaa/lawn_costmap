#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>

class LookData
{
    public:
        LookData();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber cloud_sub_;
        ros::Publisher cloud_pub_;
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
		std::string topic_name_, sensor_frame_;
};

LookData::LookData()
{
    std::cout << "start LookData" << std::endl;
	ros::NodeHandle private_nh("~");
	private_nh.param("topic_name", topic_name_, std::string("velodyne_points"));
	private_nh.param("sensor_frame", sensor_frame_, std::string("/velodyne"));
    cloud_sub_ = nh_.subscribe(topic_name_, 1, &LookData::cloudCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("min_cloud",1, true);
}

void LookData::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
{
    std::cout << "call back" << std::endl;

    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg (*msgs, pcl_cloud);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (pcl_cloud.makeShared());
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (1.8,2);
    pass.filter (pcl_cloud);

    pass.setInputCloud (pcl_cloud.makeShared());
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-2,2);
    pass.filter (pcl_cloud);

    int sum = 0;
    int intensity;
    double z;
    int freq = 30;
    std::cout << "data list" << std::endl;

	
    for(int i=0;i<pcl_cloud.width;i++){
        intensity = pcl_cloud.points[i].intensity;
        z = pcl_cloud.points[i].z;
        sum = sum + intensity;
        //if(i%freq==0)std::cout << intensity << " " << z << std::endl;
        std::cout << intensity << " " << z << std::endl;
    }
    std::cout << "ave:" << sum / pcl_cloud.width << std::endl;
    std::cout << "num:" << pcl_cloud.width << std::endl;

    sensor_msgs::PointCloud2 filtered_cloud;

    pcl::toROSMsg (pcl_cloud, filtered_cloud);
    filtered_cloud.header.frame_id = sensor_frame_;

    std::cout << "pub cloud " << std::endl;
    cloud_pub_.publish(filtered_cloud);
}



int main(int argc, char **argv)
{
    std::cout << "start look data node" << std::endl;
    ros::init(argc, argv, "look_data");
    LookData ld;
    ros::Rate r(0.2);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}


