#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <std_srvs/SetBool.h>

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
        ros::ServiceServer activate_switch_;
        tf::TransformListener tf_listener_;
		std::string sensor_frame_, topic_name_;
		int intensity_min_, intensity_max_;
		double z_min_, z_max_, remove_z_min_, remove_z_max_;
		double sensor_range_x_min_, sensor_range_x_max_, sensor_range_y_min_, sensor_range_y_max_;

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
        bool activate(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
        {
            activate_ = req.data;
            res.success = 1;
            return 1;
        }

        void publishcloud();
        void setRangeCost(costmap_2d::Costmap2D& costmap, int range, int mx, int my, int value);

        costmap_2d::Costmap2D total_costmap_;
        int activate_;
        int cost_threshold_;
};


LawnCostmap::LawnCostmap()
{

    std::cout << "start LawnCostmap" << std::endl;

	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_frame", sensor_frame_, std::string("/velodyne"));
	private_nh.param("topic_name", topic_name_, std::string("velodyne_points"));
	private_nh.param("intensity_min", intensity_min_, 45);
	private_nh.param("intensity_max", intensity_max_, 60);
	private_nh.param("z_min", z_min_, -0.6);
	private_nh.param("z_max", z_max_, -0.3);
	private_nh.param("remove_z_min", remove_z_min_, -0.3);
	private_nh.param("remove_z_max", remove_z_max_, 0.3);
	private_nh.param("sensor_range_x_min", sensor_range_x_min_, 0.0);
	private_nh.param("sensor_range_x_max", sensor_range_x_max_, 2.0);
	private_nh.param("sensor_range_y_min", sensor_range_y_min_, -2.0);
	private_nh.param("sensor_range_y_max", sensor_range_y_max_, 2.0);
	private_nh.param("cost_threshold", cost_threshold_, 3);

	std::cout << "sensor_frame:" << sensor_frame_ << std::endl;
	std::cout << "topic_name:" << topic_name_ << std::endl;
	std::cout << "intensity_min:" << intensity_min_ << std::endl;
	std::cout << "intensity_max:" << intensity_max_ << std::endl;
	std::cout << "z_min:" << z_min_ << std::endl;
	std::cout << "z_max:" << z_max_ << std::endl;
	std::cout << "remove_z_min:" << remove_z_min_ << std::endl;
	std::cout << "remove_z_max:" << remove_z_max_ << std::endl;
	std::cout << "sensor_range_x_min:" << sensor_range_x_min_ << std::endl;
	std::cout << "sensor_range_x_max:" << sensor_range_x_max_ << std::endl;
	std::cout << "sensor_range_y_min:" << sensor_range_y_min_ << std::endl;
	std::cout << "sensor_range_y_max:" << sensor_range_y_max_ << std::endl;
	std::cout << "cost_threshold:" << cost_threshold_ << std::endl;

    total_costmap_.setDefaultValue(0);
    total_costmap_.resizeMap(40, 40, 0.1, 0, 0);
   
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lawn_cloud",1, false);
    cloud_sub_ = nh_.subscribe(topic_name_, 1, &LawnCostmap::cloudCallback, this);
    activate_switch_ = nh_.advertiseService("lawn_costmap/activate", &LawnCostmap::activate, this);


    activate_ = 0;
    //activate_ = 1;

}

void LawnCostmap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs)
{

    std::cout << "cloud callback" << std::endl;

    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::PointCloud<pcl::PointXYZI> remove_pcl_cloud;
    double robot_x,robot_y;
    double new_origin_x,new_origin_y;
    unsigned int mx, my;
    double wx, wy;

    if (activate_){

 
        pcl::fromROSMsg (*msgs, pcl_cloud);

        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (sensor_range_x_min_, sensor_range_x_max_);
        pass.filter (pcl_cloud);

        std::cout << "pcl_cloud size // x filter: " << pcl_cloud.width << std::endl;

        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (sensor_range_y_min_, sensor_range_y_max_);
        pass.filter (pcl_cloud);

        std::cout << "pcl_cloud size // y filter: " << pcl_cloud.width << std::endl;
		// debug //
		
		for (int i=0;i<pcl_cloud.width;i++){
			std::cout << pcl_cloud.points[i].z << std::endl;
		}

        //remove high object
        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (remove_z_min_, remove_z_max_);
        pass.filter (remove_pcl_cloud);

		
        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (z_min_, z_max_);
        pass.filter (pcl_cloud);
	

        std::cout << "pcl_cloud size // z filter: " << pcl_cloud.width << std::endl;

		// intensity filter //
        pass.setInputCloud (pcl_cloud.makeShared());
        pass.setFilterFieldName ("intensity");
        pass.setFilterLimits (intensity_min_, intensity_max_);
        pass.filter (pcl_cloud);

        std::cout << "pcl_cloud size: " << pcl_cloud.width << std::endl;
        if(pcl_cloud.width == 0)return;

        costmap_2d::Costmap2D costmap;
        costmap.setDefaultValue(0);
        costmap.resizeMap(30,60,0.1,0,-3);

       for(int i=0;i<pcl_cloud.width;i++){
            wx = pcl_cloud.points[i].x;
            wy = pcl_cloud.points[i].y;
            if(costmap.worldToMap(wx,wy,mx,my))
                costmap.setCost(mx,my,1);
                
        }

        // height filter //
        for(int i=0;i<remove_pcl_cloud.width;i++){
           wx = remove_pcl_cloud.points[i].x;
           wy = remove_pcl_cloud.points[i].y;
           int res = costmap.worldToMap(wx,wy,mx,my);
           if(res && (costmap.getCost(mx,my)==1))
           {
               std::cout << "mx:" << mx << std::endl;
               std::cout << "my:" << my << std::endl;
               
               int sx = costmap.getSizeInCellsX();
               int sy = costmap.getSizeInCellsY();

               costmap.setCost(mx, my, 0);
               setRangeCost(costmap, 3, mx, my, 0);//range cost set

          }
        }
    

        sensor_msgs::PointCloud input_cloud;

         for(int i=0;i<costmap.getSizeInCellsX();i++){
            for(int j=0;j<costmap.getSizeInCellsY();j++){
                if(costmap.getCost(i,j)){
                    geometry_msgs::Point32 point;
                    costmap.mapToWorld(i,j,wx,wy);
                    point.x = (float)wx;
                    point.y = (float)wy;
                    input_cloud.points.push_back(point);
                }
            }
        }

        sensor_msgs::PointCloud2 cloud;
        sensor_msgs::convertPointCloudToPointCloud2(input_cloud, cloud);

        cloud.header.frame_id = sensor_frame_;
        sensor_msgs::PointCloud2 cloud_odom;

        try
        {
            tf::StampedTransform trans;
            tf_listener_.waitForTransform("odom", sensor_frame_, ros::Time(0), ros::Duration(0.5));
            tf_listener_.lookupTransform("odom", sensor_frame_, ros::Time(0), trans);
            pcl_ros::transformPointCloud("odom", trans, cloud, cloud_odom);
            pcl_ros::transformPointCloud("odom", trans, cloud, cloud_odom);
        }
        catch(tf::TransformException &e)
        {
            ROS_WARN("%s", e.what());
        }

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_odom;
        pcl::fromROSMsg (cloud_odom, pcl_cloud_odom);

        std::cout << "pcl_cloud_odom size: " << pcl_cloud_odom.width << std::endl;
        if(pcl_cloud_odom.width == 0)return;

       	//  noise filter //
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (pcl_cloud_odom.makeShared());
        sor.setMeanK (5);
        sor.setStddevMulThresh (0.1);
        sor.filter (pcl_cloud_odom);
        

        std::cout << "pcl_cloud size: " << pcl_cloud_odom.width << std::endl;
        if(pcl_cloud_odom.width == 0)return;

        unsigned int mx,my;
        double wx,wy;
        for (int i=0;i<pcl_cloud_odom.width;i++){
            wx = pcl_cloud_odom.points[i].x;
            wy = pcl_cloud_odom.points[i].y;
            if(total_costmap_.worldToMap(wx, wy, mx, my)){
                int c = total_costmap_.getCost(mx, my);
                if(c < cost_threshold_)
                    total_costmap_.setCost(mx, my, c+1);
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

    new_origin_x = robot_x - total_costmap_.getSizeInMetersX() / 2;
    new_origin_y = robot_y - total_costmap_.getSizeInMetersY() / 2;
    total_costmap_.updateOrigin(new_origin_x, new_origin_y);

    publishcloud();
}


void LawnCostmap::setRangeCost(costmap_2d::Costmap2D& costmap, int range, int mx, int my, int value){

    int limit_x=mx,limit_nx=mx,limit_y=my,limit_ny=my;
    int size_x = costmap.getSizeInCellsX();
    int size_y = costmap.getSizeInCellsY();
    for(int i=0;i<range;i++){
        limit_x++;
        limit_y++;
        limit_nx--;
        limit_ny--;
        if(limit_x > size_x)limit_x = size_x;
        if(limit_y > size_y)limit_y = size_y;
        if(limit_nx < 0)limit_nx = 0;
        if(limit_ny < 0)limit_ny = 0;
    }

    int num_x = limit_x - limit_nx + 1;
    int num_y = limit_y - limit_ny + 1;

    unsigned int x,y;
    
    for(int i=0;i<num_x;i++){
        for(int j=0;j<num_y;j++){
            x = limit_nx+i;
            y = limit_nx+j;
            costmap.setCost(x,y,0);
        }
    }
}

void LawnCostmap::publishcloud()
{

    double wx,wy;
    sensor_msgs::PointCloud map_cloud_odom;
    sensor_msgs::PointCloud2 map_cloud2_odom;
    sensor_msgs::PointCloud2 map_cloud2_velodyne;

    
    for(int i=0;i<total_costmap_.getSizeInCellsX();i++){
        for(int j=0;j<total_costmap_.getSizeInCellsY();j++){
            if(total_costmap_.getCost(i,j) > cost_threshold_ - 1){
                geometry_msgs::Point32 point;
                total_costmap_.mapToWorld(i,j,wx,wy);
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
        tf_listener_.waitForTransform("odom", sensor_frame_, ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform(sensor_frame_, "odom", ros::Time(0), trans);
        pcl_ros::transformPointCloud(sensor_frame_, trans, map_cloud2_odom, map_cloud2_velodyne);
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
