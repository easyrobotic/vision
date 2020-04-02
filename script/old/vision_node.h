#ifndef ros_img_processor_node_H
#define ros_img_processor_node_H

//std C++
#include <iostream>

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//ROS headers for image I/O
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

/** \Turtlebot image processor 2D and 3D
 *
 * Simple Image Processor with opencv calls
 *
 */

class VisionNode
{
  protected:
    ros::NodeHandle nh;
  public:
    VisionNode();
    ~VisionNode();
    2Ddetection();
}



/*class RosImgProcessorNode
{
    protected:
        //ros node handle
        ros::NodeHandle nh_;

        //image transport
        image_transport::ImageTransport img_tp_;

        // subscribers to the image and camera info topics
        image_transport::Subscriber image_subs_;
        ros::Subscriber camera_info_subs_;
        ros::Subscriber Kalmann_Filter_;

        //publishers
        image_transport::Publisher image_pub_;
        ros::Publisher marker_points_;
		    ros::Publisher marker_publisher_;
        //publishing state space
        ros::Publisher raw_data_;


        //pointer to received (in) and published (out) images
        cv_bridge::CvImagePtr cv_img_ptr_in_;
        cv_bridge::CvImage cv_img_out_;

		    //Camera matrix
		    Eigen::Matrix3d matrixK_;
	    	Eigen::Vector3d direction_;
        Eigen::Vector3d center_3d_;

        //Output image

        cv::Point center_kallman_;//kallman

        cv::Mat output_image_;

        //Center
        cv::Point center_;//

        //radius
        int radius_;
        Eigen::Vector3d X_;

        //image encoding label
        std::string img_encoding_;

        //wished process rate, [hz]
        double rate_;

        //time
        ros::Time time_raw_data_;

    protected:
        // callbacks
        void imageCallback(const sensor_msgs::ImageConstPtr& _msg);
        void cameraInfoCallback(const sensor_msgs::CameraInfo & _msg);
        void KalmanFilterCallback(const geometry_msgs::Vector3& _msg);

    public:
        // Constructor
        RosImgProcessorNode();

        // Destructor
        ~RosImgProcessorNode();

        // Process input image
        void process();

        // Getting direction values
        void getMarker();

        //Getting state state_space
        void stateSpace();

        // Publish output image
        void publishImage();

		// Publish direction marker
        void publishMarker();

 		// Returns rate_
        double getRate() const;

};
#endif
*/
