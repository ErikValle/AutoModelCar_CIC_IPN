#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <math.h>
#include <numeric>
#include "std_msgs/Int16.h"

int LINE_WIDTH=8;
int TOL=4;

int dist;
int ang;
std_msgs::Int16 p_ang,p_dist;

class ImageConverter
{
  	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
  	image_transport::Publisher image_pub_;
	ros::Publisher PubAng;
	ros::Publisher PubDist;

public:
  	ImageConverter()
    	: it_(nh_)
  {
    	image_sub_ = it_.subscribe("/img_prepros", 1,
      	&ImageConverter::imageCb, this,image_transport::TransportHints("compressed"));
    	image_pub_ = it_.advertise("/inter_line", 1);

	PubAng= nh_.advertise<std_msgs::Int16>("inter_line/ang",1);
	PubDist= nh_.advertise<std_msgs::Int16>("inter_line/dist",1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    	cv_bridge::CvImagePtr cv_ptr;
    	sensor_msgs::ImagePtr sal_ptr;
    try
    {
      	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }

	int e1=cv::getTickCount();
//La imagen contenida en cv_ptr se guarda en una matriz 
   	cv::Mat image=cv_ptr->image;

//Reescala
   	cv:: Mat scld;
   	cv::resize(image, scld, cv::Size(), 0.6, 0.6,CV_INTER_LINEAR);

//Binarización
    cv::Mat bin;
    cv::inRange(scld,140,190,bin );

	int wdth, hght,i,j,point,cont;
	std::vector<cv::Point>  ini;
	std::vector<int> pc;
	wdth=bin.cols;
	hght=bin.rows;

	i=hght-2;
	j=wdth*0.35;
	point=hght-1;
	while (j<(wdth*0.8))
	{

		cont=0;
		pc.erase(pc.begin(),pc.end());
		while ((hght*0.5<i)&&(i<hght))
		{
			if (bin.at<uchar>(i,j)!=0)
			{
				cont++;
				pc.push_back(i);
			}
			else
			{
				if ((LINE_WIDTH-TOL<cont)&&(cont<LINE_WIDTH+TOL))
				{
					if (abs(point-pc[0])<7)
					{
						ini.push_back(cv::Point2f(j,pc[0]));
						cv::circle(scld,cv::Point(j,pc[0]),1,250,-1);
                                                
		
					}
					point = pc[0]+TOL;
					i = 0;
				}
				else
				{
					cont=0;
					pc.erase(pc.begin(),pc.end());
				}
				
			}
			i--;
		}
		i = point;
		j += 2;
	}
int x=ini.size();
if (ini.size()>13){
	cv::Vec4f p;
	cv::fitLine(ini, p,CV_DIST_L1,0,0.01,0.01);
	cv::circle(scld,cv::Point(p[2],p[3]),1,0,-1);
	cv::line(scld,cv::Point(p[2],p[3]),cv::Point(p[2]+p[0]*50,p[3]+p[1]*50),250);
	int ang=(57*atan(p[1]/p[0]));
	int dist=(hght-p[3]);	
	p_ang.data=ang;
	p_dist.data=dist;
	ROS_INFO("dist: %i	|| ang: %i",dist,ang);

}
else {
        p_dist.data=200;
        PubDist.publish(p_dist);
     }
if ((ang>-15)&&(ang<15))
{
	PubAng.publish(p_ang);
	PubDist.publish(p_dist);
}


//ROS_INFO("dist: %i       || ang: %i",dist,ang);
cv_bridge::CvImage out_msg;
out_msg.header   = msg->header; // Same timestamp and tf frame as input image
out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
out_msg.image    = scld; // Your cv::Mat
sal_ptr=out_msg.toImageMsg();
image_pub_.publish(sal_ptr);
int e2=cv::getTickCount();
float t=(e2-e1)/cv::getTickFrequency();
ROS_INFO("frame time: %f-------------------------------block end", t);
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "inter_detect");
  ImageConverter ic;
  ros::spin();
  return 0;
}
