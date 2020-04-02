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
#include <visualization_msgs/Marker.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;


ros::Publisher pub;
ros::Publisher pub_cloud_transformed;
PointCloudC::Ptr cloud(new PointCloudC());
tf::StampedTransform transform;
ros::Publisher marker_pub;
ros::Publisher pub_persones;

 geometry_msgs::PointStamped persones_points [6];
 geometry_msgs::PointStamped perill_points [6];
 geometry_msgs::PointStamped sortida_points [1];

void markers_rviz (geometry_msgs::PointStamped pointmark [], std::string color_def){//tf::StampedTransform transform){

  visualization_msgs::Marker marker;

  for (int i=1; i < 6; i++){
    if (pointmark[i].point.x != 0){
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(0);
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = i;
  marker.type = visualization_msgs::Marker::CUBE;
//  marker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pointmark[i].point.x;
  marker.pose.position.y = pointmark[i].point.y;
  marker.pose.position.z = pointmark[i].point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.5;
  // Set the color -- be sure to set alpha to something non-zero!
      if (color_def=="blue"){
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
      }
      if (color_def=="green"){
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
      }
      if (color_def=="red"){
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
      }
  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
  }
}

}

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
     geometry_msgs::PointStamped punt_transformat;

     tf::TransformListener listener;
     ros::Rate rate(10.0);
    try{
         listener.waitForTransform("base_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
         listener.transformPoint("map", centroidPointb, punt_transformat);
         std::cout << "Listener a base_link : " << punt_transformat << std::endl;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }


if (centroidPointb.point.x != 0 and centroidPointb.point.y != 0) {
      //BLAAAAAUUUUU
      if ((centroidPointb.point.x != 0) and (color_def=="blue")){
        std::cout <<"PERSONAAA!!! (en funció camera_depth_optical_frame)  X: " << centroidPointb.point.x <<" Y: " << centroidPointb.point.y << " Z: " << centroidPointb.point.z << "\n";

         for (int i=1; i < 6; i++){
           std::cout << "Points persones " << i << ": " << persones_points[i].point << std::endl;

           if (persones_points[1].point.x == 0 and persones_points[1].point.y == 0){
             persones_points[1].point.x = punt_transformat.point.x;
             persones_points[1].point.y = punt_transformat.point.y;
             persones_points[1].point.z = punt_transformat.point.z;
             }

         if ((persones_points[i].point.x != 0) and ((punt_transformat.point.x-persones_points[i].point.x>abs(0.25)) or (punt_transformat.point.y-persones_points[i].point.y > abs(0.25)))) //(punt_transformat.point.x-persones_points[i].point.x>0.3) or (punt_transformat.point.x-persones_points[i].point.x < -0.3) or (punt_transformat.point.y-persones_points[i].point.y < -0.3) or (punt_transformat.point.y-persones_points[i].point.y > 0.3)))
         {
             persones_points[i+1].point.x = punt_transformat.point.x;
             persones_points[i+1].point.y = punt_transformat.point.y;
             persones_points[i+1].point.z = punt_transformat.point.z;
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
             }

         if ((perill_points[i].point.x != 0) and ((punt_transformat.point.x-perill_points[i].point.x>abs(0.25)) or (punt_transformat.point.y-perill_points[i].point.y > abs(0.25)))) //(punt_transformat.point.x-persones_points[i].point.x>0.3) or (punt_transformat.point.x-persones_points[i].point.x < -0.3) or (punt_transformat.point.y-persones_points[i].point.y < -0.3) or (punt_transformat.point.y-persones_points[i].point.y > 0.3)))
         {
             perill_points[i+1].point.x = punt_transformat.point.x;
             perill_points[i+1].point.y = punt_transformat.point.y;
             perill_points[i+1].point.z = punt_transformat.point.z;
             }
         }
      }



      if ((centroidPointb.point.x != 0) and (color_def == "green")){
        std::cout <<"SORTIDA!!! (en funció camera_depth_optical_frame)  X: " << centroidPointb.point.x <<" Y: " << centroidPointb.point.y << " Z: " << centroidPointb.point.z << "\n";

         for (int i=1; i < 6; i++){
  std::cout << "Points sortida " << i << ": " << sortida_points[i].point << std::endl;

           if (sortida_points[1].point.x == 0 and sortida_points[1].point.y == 0){
             sortida_points[1].point.x = punt_transformat.point.x;
             sortida_points[1].point.y = punt_transformat.point.y;
             sortida_points[1].point.z = punt_transformat.point.z;
             }
         }

      }

// Visualitzem markers a RVIZ
markers_rviz(persones_points,"blue");
markers_rviz(perill_points,"red");
markers_rviz(sortida_points,"green");

}
    //  pub_persones.publish(*persones_points);
      //sensor_msgs::PointCloud2 msg_out;
      //pcl::toROSMsg(*cloud_color, msg_out);
      //pub.publish(msg_out);

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
//function_cloud(cropped_cloud, "green");
//function_cloud(cropped_cloud, "red");


  //conditional remover color red
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_red(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter_red;
  // pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr red_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, 60));
  // pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond_red (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  // color_cond_red->addComparison (red_condition);
  // color_filter_red.setInputCloud(cropped_cloud);
  // color_filter_red.setCondition (color_cond_red);
  // color_filter_red.filter(*cloud_red);

   //conditional remover color gren
   //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_green ( new pcl::PointCloud<pcl::PointXYZRGB>);
  //  pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter_green;
  //  pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr green_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, 80));
  //  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond_green (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  //  color_cond_green->addComparison (green_condition);
  //  color_filter_green.setInputCloud(cropped_cloud);
  //  color_filter_green.setCondition (color_cond_green);
  //  color_filter_green.filter(*cloud_green);


//PointC persona[5] = {};

//int verd = cloud_green->points[cloud_green->size()/2].g;
//int vermell = cloud_red->points[cloud_red->size()/2].r;


 //if ((centroidPoint.x != 0)and(vermell > 90)){
//  std::cout <<"PERILL!!!   X: " << centroidPoint.x <<" Y: " << centroidPoint.y << "\n";
//}
//else if ((centroidPoint.x != 0)and(verd > 90)){
//  std::cout <<"SORTIDA!!!   X: " << centroidPoint.x <<" Y: " << centroidPoint.y << "\n";
//}


// Crear topic PointCloud2 amb la informació filtrada del cropping
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtrat_tot;
//pcl::concatenateFields(cloud_blue, cloud_green, cloud_filtrat_tot);


}




main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "astra_detection");
  ros::NodeHandle nh;

  // Subscriber al cloud de la camera astra
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
  // Publisher cloud filtres i tal
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_astra_detection", 1);
  // Publisher marker RVIZ
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  pub_persones = nh.advertise<geometry_msgs::Point>("persones_point_output",1);

  //Definició listener transformades
  tf::TransformListener listener;
  ros::Rate rate(30.0);
    while (nh.ok()){

        try{
          listener.waitForTransform( "map", "camera_depth_optical_frame",  ros::Time(0), ros::Duration(4.0));
          listener.lookupTransform( "map", "camera_depth_optical_frame",  ros::Time(0), transform);
      //      std::cout << "Entrem en el listener x:  " << transform.getOrigin().x() << "\n";
      //      std::cout << "Entrem en el listener y:  " << transform.getOrigin().y() << "\n";
      //      std::cout << "Entrem en el listener z:  " << transform.getOrigin().z() << "\n";
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        //markers_rviz(transform);
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;


}
