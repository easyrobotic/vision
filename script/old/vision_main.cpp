#include "vision_node.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "astra_detection");

  //create ros wrapper object
  //set node loop rate
  ros::Rate loopRate(imgp.getRate());

  //node loop
  while (ros::ok() )
  {

  }

  //exit program
  return 0;
}
/*
#include "ros_img_processor_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "ros_img_processor");

      //create ros wrapper object
      RosImgProcessorNode imgp;

      //set node loop rate
      ros::Rate loopRate(imgp.getRate());

      //node loop
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce();

            //do things
            imgp.process();

            //publish things
            imgp.publishImage();
			      imgp.publishMarker();
            imgp.getMarker();
            //imgp.stateSpace();
            //relax to fit output rate
            loopRate.sleep();
      }

      //exit program
      return 0;
}
*/
