#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/common.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/conditional_removal.h>
#include <math.h>
#include <sstream>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;


ros::Publisher pub;
ros::Publisher pub_cloud_transformed;
PointCloudC::Ptr cloud(new PointCloudC());
tf::StampedTransform transform;
ros::Publisher pub_persones;
ros::Publisher pub_perill;
ros::Publisher pub_sortida;

 geometry_msgs::PointStamped persones_points [6];
 geometry_msgs::PointStamped perill_points [6];
 geometry_msgs::PointStamped sortida_points [1];


void function_cloud(PointCloudC::Ptr& cropped_cloud, std::string color_def)
{

    //conditional removal color blau
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
     pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter_color;

     pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr color_condition_b(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, 85));
     pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr color_condition_g(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, 70));
     pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr color_condition_r(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, 60));

     //aplicar filtre
     pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond_color (new pcl::ConditionAnd<pcl::PointXYZRGB> ());

      if (color_def == "blue"){
       color_cond_color->addComparison (color_condition_b);}
      if (color_def == "red"){
       color_cond_color->addComparison (color_condition_r);}
      if (color_def == "green"){
       color_cond_color->addComparison (color_condition_g);}

     color_filter_color.setInputCloud(cropped_cloud);
     color_filter_color.setCondition (color_cond_color);
     color_filter_color.filter(*cloud_color);

     //Càlcul punt mig del pointcloud filtrat per colors.
     PointC min_pclb;
     PointC max_pclb;
     pcl::getMinMax3D<PointC>(*cloud_color, min_pclb, max_pclb);
     geometry_msgs::PointStamped centroidPointb;
     centroidPointb.point.x = (min_pclb.x + max_pclb.x)/2;
     centroidPointb.point.y = (min_pclb.y + max_pclb.y)/2;
     centroidPointb.point.z = (min_pclb.z + max_pclb.z)/2;
     centroidPointb.header.frame_id = "camera_depth_optical_frame";
     centroidPointb.header.stamp = ros::Time(0);
     std::cout << "Valors centroid::  " << centroidPointb <<std::endl;

     //Transformar punt camera a coordenades map


     tf::TransformListener listener_;
     ros::Rate rate(10.0);



if (centroidPointb.point.x != 0 and centroidPointb.point.y != 0) {
       geometry_msgs::PointStamped punt_transformat;
  try{

       listener_.waitForTransform("camera_dept_optical_frame", "map", ros::Time(0), ros::Duration(1.0));
       listener_.transformPoint("map", centroidPointb, punt_transformat);
       std::cout << "Listener a base_link : " << punt_transformat << std::endl;
     }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

      //BLAAAAAUUUUU
      if ((centroidPointb.point.x != 0) and (color_def=="blue")){
        std::cout <<"PERSONAAA!!! (en funció camera_depth_optical_frame)  X: " << centroidPointb.point.x <<" Y: " << centroidPointb.point.y << " Z: " << centroidPointb.point.z << "\n";

         for (int i=1; i < 6; i++){
           std::cout << "Points persones " << i << ": " << persones_points[i].point << std::endl;

           if (persones_points[1].point.x == 0 and persones_points[1].point.y == 0){
             persones_points[1].point.x = punt_transformat.point.x;
             persones_points[1].point.y = punt_transformat.point.y;
             persones_points[1].point.z = punt_transformat.point.z;
             //pub_persones.publish(persones_points[1]);
             }

         if ((persones_points[i].point.x != 0) and ((punt_transformat.point.x-persones_points[i].point.x>abs(0.25)) or (punt_transformat.point.y-persones_points[i].point.y > abs(0.25)))) //(punt_transformat.point.x-persones_points[i].point.x>0.3) or (punt_transformat.point.x-persones_points[i].point.x < -0.3) or (punt_transformat.point.y-persones_points[i].point.y < -0.3) or (punt_transformat.point.y-persones_points[i].point.y > 0.3)))
         {
             persones_points[i+1].point.x = punt_transformat.point.x;
             persones_points[i+1].point.y = punt_transformat.point.y;
             persones_points[i+1].point.z = punt_transformat.point.z;
             //pub_persones.publish(persones_points[i+1]);
             }

        if ((persones_points[i].point.x != 0) and  (persones_points[i].point.y != 0)){
          pub_persones.publish(persones_points[i]);
        }

      }
    }

      //VERMEEEELLL
      if ((centroidPointb.point.x != 0) and (color_def=="red")){
        std::cout <<"PERILL!!! (en funció camera_depth_optical_frame)  X: " << centroidPointb.point.x <<" Y: " << centroidPointb.point.y << " Z: " << centroidPointb.point.z << "\n";

         for (int i=1; i < 6; i++){
           std::cout << "Points perill " << i << ": " << perill_points[i].point << std::endl;

           if (perill_points[1].point.x == 0 and perill_points[1].point.y == 0){
             perill_points[1].point.x = punt_transformat.point.x;
             perill_points[1].point.y = punt_transformat.point.y;
             perill_points[1].point.z = punt_transformat.point.z;
             //pub_perill.publish(perill_points[1]);

             }

         if ((perill_points[i].point.x != 0) and ((punt_transformat.point.x-perill_points[i].point.x>abs(0.25)) or (punt_transformat.point.y-perill_points[i].point.y > abs(0.25)))) //(punt_transformat.point.x-persones_points[i].point.x>0.3) or (punt_transformat.point.x-persones_points[i].point.x < -0.3) or (punt_transformat.point.y-persones_points[i].point.y < -0.3) or (punt_transformat.point.y-persones_points[i].point.y > 0.3)))
         {
             perill_points[i+1].point.x = punt_transformat.point.x;
             perill_points[i+1].point.y = punt_transformat.point.y;
             perill_points[i+1].point.z = punt_transformat.point.z;
             //pub_perill.publish(perill_points[i+1]);
             }


         if ((perill_points[i].point.x != 0) and  (perill_points[i].point.y != 0)){
           pub_persones.publish(perill_points[i]);
         }
         }
      }



      if ((centroidPointb.point.x != 0) and (color_def == "green")){
        std::cout <<"SORTIDA!!! (en funció camera_depth_optical_frame)  X: " << centroidPointb.point.x <<" Y: " << centroidPointb.point.y << " Z: " << centroidPointb.point.z << "\n";


           if (sortida_points[1].point.x == 0 and sortida_points[1].point.y == 0){
             sortida_points[1].point.x = punt_transformat.point.x;
             sortida_points[1].point.y = punt_transformat.point.y;
             sortida_points[1].point.z = punt_transformat.point.z;
             pub_sortida.publish(sortida_points[1]);
             }


      }


}


}

void cloud_cb (const sensor_msgs::PointCloud2& msg)
{
//  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
//  ROS_INFO("Got point cloud with %ld points", cloud->size());


//CROPPING --Per eliminar soroll. Elimina punts que llegim de la càmera per centrar l'atenció en una àrea en concret.
    PointCloudC::Ptr cropped_cloud(new PointCloudC());
    Eigen::Vector4f min_pt(-2.5, -1, 0.5, 1);
    Eigen::Vector4f max_pt(2.5, 1, 5, 1);
    pcl::CropBox<PointC> crop;
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cropped_cloud);
  //ROS_INFO("Cropped to %ld points", cropped_cloud->size());
  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*cropped_cloud, min_pcl, max_pcl);
  geometry_msgs::Point centroidPoint;
  centroidPoint.x = (min_pcl.x + max_pcl.x)/2;
  centroidPoint.y = (min_pcl.y + max_pcl.y)/2;
  std::cout << "Header point cloud camera" << cropped_cloud->header <<std::endl;

///
function_cloud(cropped_cloud, "blue");
function_cloud(cropped_cloud, "green");
function_cloud(cropped_cloud, "red");


}




main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "astra_detection");
  ros::NodeHandle nh;

  // Subscriber al cloud de la camera astra
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
  ros::Subscriber sub_ = nh.subscribe ("/astra/depth/points", 1, cloud_cb);

  // Publisher cloud filtres i tal
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_astra_detection", 1);
  // Publisher marker RVIZ


  pub_persones = nh.advertise<geometry_msgs::PointStamped>("vision/astra/blue_pose",1);
  pub_perill = nh.advertise<geometry_msgs::PointStamped>("vision/astra/red_pose",1);
  pub_sortida = nh.advertise<geometry_msgs::PointStamped>("vision/astra/green_pose",1);


    ros::spin();

    return 0;


}
