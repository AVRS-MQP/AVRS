#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <fstream>

#include "cv_service_msgs/cv_service.h"

using namespace std;
using namespace cv;

int mode = 0;

int LOOP_TIME = 8;

float rawX = 0;
float rawY = 0;
int imgX = 0;
int imgY = 0;

int filter = 7;
int t_hold = 100;
int circlet = 65;

float sumX = 0;
float sumY = 0;
int loop = 0;

float avgX = 0;
float avgY = 0;

image_transport::Publisher pub2;
image_transport::Publisher pub3;

class Cv_service{
  public:
    static cv::Mat getCircles(cv::Mat img){
      vector<Vec3f> circles;
//	ros::Duration(.1).sleep();//sleep for a little
      // Apply the Hough Transform to find the circles
      printf("circlet is %d\n", circlet);
      printf("mode is %d\n", mode);
      HoughCircles(img, circles, CV_HOUGH_GRADIENT, 1, img.rows/8, 200, circlet, 0, 0);

      cvtColor(img, img, CV_GRAY2BGR );

      if(circles.size() >= 1){
	rawX = cvRound(circles[0][0]);
	rawY = cvRound(circles[0][1]);

	sumX += cvRound(circles[0][0]);
	sumY += cvRound(circles[0][1]);
      } else{
	printf("404 FLAP NOT FOUND\n");
	loop--;  //dont count instances when flap is not found
	return img;
      }

      if(loop >= LOOP_TIME){
	printf("Publishing avg image\n");
	cv::Mat meanImg = img;
	float deltaX = sumX/loop;
	float deltaY = sumY/loop;

	Point center(deltaX, deltaY);
	printf("Test: deltax is %f, deltay is %f\n", avgX, avgY);
	circle(meanImg, center, 3, Scalar(255,255,0), -1, 8, 0 );

	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv_ptr->encoding = "bgr8";
	cv_ptr->image = meanImg;

	pub3.publish(cv_ptr->toImageMsg());

	//imshow("average flap center", meanImg);
	//waitKey(4010);
      }

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

    static cv::Mat findFlap(cv::Mat img){

      cv::Mat detected_edges;

      //clean up
      blur(img, detected_edges, Size(filter, filter) );

      //convert to edges

      //imshow( "b/w blur", detected_edges);

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
    static cv::Mat imageTransform(cv::Mat img){

      //convert to grayscale
      cvtColor(img, img, CV_BGR2GRAY );

      //add blur
      //GaussianBlur(img, img, Size(9, 9), 2, 2);

      return img;
    }

    //FOR CAMERA USE
    void getLiveImage(const sensor_msgs::ImageConstPtr& msg){

      //printf("test\n");
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;


      //printf("test2\n");
      // node handler
      ros::NodeHandle n;
      //ros::Publisher pub = n.advertise<geometry_msgs::Pose>("rawCV", 1);

      if(!image.data){
	printf("oh no\n");
	return;
      }

      imgX = image.size().width;
      imgY = image.size().height;

      printf("Width and height are %d, %d\n", imgX, imgY);

      loop++;

      image = imageTransform(image);
      image = findFlap(image);

      printf("loop =                    %d\n", loop);
      if(loop >= LOOP_TIME){
	avgX = sumX/imgX/loop;
	avgY = sumY/imgY/loop;

	sumX = 0;
	sumY = 0;
	loop = 0;
	printf("reset\n");
      }


      //publish image to ros topic 
      //sensor_msgs::Image imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg;
      cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
      cv_ptr->encoding = "bgr8";
      cv_ptr->image = image;

      pub2.publish(cv_ptr->toImageMsg());		
      //	while(ros::ok){
      //pub2.publish(imgmsg);
      //		}


      //pub.publish(pose);

      //ros::spin();
    }

    static void getTestImage(){
      ros::NodeHandle n;
      //ros::Publisher pub = n.advertise<geometry_msgs::Pose>("rawCV", 1);

      cv::Mat img = imread("test_image_2.png");


      imgX = img.size().width;
      imgY = img.size().height;

      img = imageTransform(img);
      img = findFlap(img);

      /*geometry_msgs::Pose msg;
	msg.position.x = rawX/imgX;
	msg.position.y = rawY/imgY;*/

      //pub.publish(msg);

      cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
      cv_ptr->encoding = "bgr8";
      cv_ptr->image = img;
      pub2.publish(cv_ptr->toImageMsg());		
    }

    bool cvCB(cv_service_msgs::cv_service::Request  &req, 
	cv_service_msgs::cv_service::Response &res){//publically accessible class to handle service callback
   
/*  
      ros::NodeHandle n;

      //Live Image
      if(mode == 0){
	while(loop <= 10){
	  loop++;
	}

	printf("Loop = %d\n", loop);
	printf("sumX = %f\n", sumX);
	printf("sumY = %f\n", sumY);
	float deltaX = sumX/imgX/loop;
	printf("deltaX = %f\n", deltaX);
	float deltaY = sumY/imgY/loop;
	printf("deltaY = %f\n", deltaY);
      } 
      else{
	while(loop <= 10){
	  loop++;
	  getTestImage();
	  sleep(1);
	}

	printf("Loop = %d\n", loop);
	printf("sumX = %f\n", sumX);
	printf("sumY = %f\n", sumY);
	float deltaX = sumX/imgX/loop;
	printf("deltaX = %f\n", deltaX);
	float deltaY = sumY/imgY/loop;
	printf("deltaY = %f\n", deltaY);
      }
*/
      res.flapX = avgX;
      res.flapY = avgY;
      res.status = 1;

      return true;
    }

};


int main(int argc, char **argv){
  //initialize node
  ros::init(argc, argv, "cv_node");

  // node handler
  ros::NodeHandle n;

  //load setting from .yaml
  n.getParam("settings/mode", mode);
  n.getParam("settings/filter", filter);
  n.getParam("settings/t_hold", t_hold);
  n.getParam("settings/circlet", circlet);

  Cv_service CV;

  image_transport::ImageTransport it(n);
  image_transport::ImageTransport it2(n);
  pub2 = it.advertise("arm_camera/image", 1);
  pub3 = it2.advertise("arm_camera/average", 1);

  ros::ServiceServer service = n.advertiseService("cv_service", &Cv_service::cvCB, &CV);//tie service to callback function inside a class

  ros::Subscriber sub = n.subscribe("cv_camera/image_raw", 1, &Cv_service::getLiveImage, &CV);//ARMCAM: cv_camera/image_raw, Kinect:/camera/rgb/image_color
  ros::spin();

  return 0;
}
