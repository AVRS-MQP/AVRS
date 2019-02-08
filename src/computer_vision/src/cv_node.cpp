#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <fstream>

//cd ~/catkin_ws
//rosrun opencv_example opencv_example_node

using namespace std;
using namespace cv;

int mode = 1;

float rawX = 0;
float rawY = 0;

int filter = 7;
int t_hold = 102;
int circlet = 65;

cv::Mat getCircles(cv::Mat img){
  vector<Vec3f> circles;

  // Apply the Hough Transform to find the circles
  HoughCircles(img, circles, CV_HOUGH_GRADIENT, 1, img.rows/8, 200, circlet, 0, 0);

  cvtColor(img, img, CV_GRAY2BGR );

  //rawX = cvRound(circles[0][0]);
  //rawY = cvRound(circles[0][1]);

  // Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);

      printf("%d, %d, %d\n", cvRound(circles[i][0]), cvRound(circles[i][1]), cvRound(circles[i][2]));
      // circle center
      circle(img, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle(img, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }

  printf("%lu\n", circles.size());

  return img;
}

cv::Mat findFlap(cv::Mat img){

  cv::Mat detected_edges;

  //clean up
  blur(img, detected_edges, Size(filter, filter) );

  //convert to edges

  imshow( "b/w blur", detected_edges);

  Canny(detected_edges, detected_edges, t_hold, t_hold*3, 5);

  /// Using Canny's output as a mask, we display our result
  Mat dst;
  dst = Scalar::all(0);

  img.copyTo( dst, detected_edges);
  //imshow( window_name, dst );

  dst = getCircles(dst);

  return dst;
}


//Make any neccessary transformations
cv::Mat imageTransform(cv::Mat img){

  //convert to grayscale
  cvtColor(img, img, CV_BGR2GRAY );

  //add blur
  //GaussianBlur(img, img, Size(9, 9), 2, 2);

  return img;
}

//FOR CAMERA USE
void getLiveImage(const sensor_msgs::ImageConstPtr& msg)
{

  printf("test\n");
  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;


  printf("test2\n");
  // node handler
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Pose>("rawCV", 1);

  if(!image.data){
    printf("oh no\n");
    return;
  }

  float imgW = image.size().width;
  float imgH = image.size().height;
 
  printf("%f, %f\n", imgW, imgH);

  image = imageTransform(image);
  image = findFlap(image);

  /*geometry_msgs::Pose pose;
  pose.position.x = rawX/imgW;
  pose.position.y = rawY/imgH;*/

  imshow("flap", image);
  waitKey(0);

  //pub.publish(pose);

  //ros::spin();
}


int main(int argc, char **argv)
{
 
  //initialize node
  ros::init(argc, argv, "cv_node");

  // node handler
  ros::NodeHandle n;

  //publisher
  ros::Publisher pub = n.advertise<geometry_msgs::Pose>("rawCV", 1);


  if(mode == 0){
    // subsribe topic
    ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 1, getLiveImage);

    ros::spin();
  } else{
    //FOR SAVED IMAGE

    cv::Mat img = imread("test_image_2.png");

    float imgW = img.size().width;
    float imgH = img.size().height;
    printf("%f, %f\n", imgW, imgH);

    img = imageTransform(img);
    img = findFlap(img);

    printf("%f, %f\n", rawX/imgW, rawY/imgH);

    geometry_msgs::Pose msg;
    msg.position.x = rawX/imgW;
    msg.position.y = rawY/imgH;

    pub.publish(msg);

    imshow( "image", img);
    waitKey(0);

  }

  return 0;
}
