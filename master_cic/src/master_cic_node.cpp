#include <ros/ros.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

float KEEP_DIST=0.8;
bool stopFlag= false;
bool passFlag=false;
int cnt=0;
std_msgs::Int16 lineFollowSteering,lineFollowSpeed,lin_ang,lin_angs,angDir,vel,lin_dist;
std_msgs::Float32 lin_dists;
std_msgs::String lights;
class Master
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_; 
ros::Publisher pubDir,pubVel,pubLight;
ros::Subscriber steering_top,speed_top,linang_top,lindist_top,linang_tops,lindist_tops;
public:
    Master() 
    : it_(nh_)  
  {
     image_sub_ = it_.subscribe("/img_prepros", 1,&Master::imageCb, this);
     image_pub_ = it_.advertise("/master_cic", 1); 
     steering_top = nh_.subscribe("/lane_follower/steering", 1, &Master::steering_cb, this);
     speed_top= nh_.subscribe("/lane_follower/speed", 1, &Master::speed_cb, this);
     linang_top= nh_.subscribe("inter_line/ang",1,&Master::linang_cb, this);
     lindist_top=nh_.subscribe("inter_line/dist",1,&Master::lindist_cb, this);
     linang_tops= nh_.subscribe("/scan_followc/angle",1,&Master::linangs_cb, this);
     lindist_tops=nh_.subscribe("/scan_followc/dist",1,&Master::lindists_cb, this);
     pubDir= nh_.advertise<std_msgs::Int16>("/manual_control/steering",1);//Para publicar en consola
     pubVel= nh_.advertise<std_msgs::Int16>("/manual_control/speed",1);
     pubLight= nh_.advertise<std_msgs::String>("/manual_control/lights",1);
  }

int velCatch(std_msgs::Int16 vel,std_msgs::Int16 cval) 
{
 
    if(vel.data>cval.data+9)
    {
      vel.data=vel.data-2;
    }
    else if(vel.data<cval.data)
    {
      vel.data=vel.data+6;
    }
    else if(cval.data==0)
    {
      vel.data=0;
    }
    return vel.data;
}
int controlDist(std_msgs::Float32 lin_dists)
{
  int velCalc;

  if(lin_dists.data>KEEP_DIST+0.1)
  {
    velCalc= int(lin_dists.data*(-300));
  }
  else if (lin_dists.data<KEEP_DIST-0.1 && lin_dists.data>0.2)
  {
    velCalc= int(1/lin_dists.data*(50)); 
  }
  else if(KEEP_DIST-0.1<lin_dists.data && lin_dists.data<KEEP_DIST+0.1)
  {
    velCalc=0;
  }
  else
  {
    velCalc=0;
  }
  return velCalc;
}
void steering_cb(const std_msgs::Int16 steering) 
  {  
    lineFollowSteering.data = steering.data;
   // ROS_INFO("%i",lineFollowSteering.data);
  }
void speed_cb(const std_msgs::Int16 speed) 
  {  
     lineFollowSpeed.data = speed.data;
     //ROS_INFO("%i",lineFollowSpeed.data);
  }
void linang_cb(const std_msgs::Int16 linang) 
  {  
     lin_ang.data = linang.data;
  }
void lindist_cb(const std_msgs::Int16 lindist) 
  {  
     lin_dist.data=lindist.data;
   //  ROS_INFO("%i",lin_dist.data);
  }
void linangs_cb(const std_msgs::Int16 linangs) 
  { 
     lin_angs.data=linangs.data;
  }
void lindists_cb(const std_msgs::Float32 lindists) 
  {  
     lin_dists.data=lindists.data; 
  }

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    	cv_bridge::CvImagePtr cv_ptr;
    try
    	{
     	cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
    	}
    catch (cv_bridge::Exception& e) 
    	{
      	ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
      }
       int e1=cv::getTickCount();
///////////////////////Color transform///////////////////////////////
       cv::Mat image=cv_ptr->image;
       
     //  lights.data="diL";
       //angDir.data=98;
       //vel.data=0;
       //lineFollowSteering.data=100;
       //lineFollowSpeed.data=0;
       //lin_dists.data=0.1;
       //lin_angs.data=220;
       //lin_dist.data=100;
       //lin_ang.data=180;
/////////////////////////Overcoming//////////////////////////
       
   if (passFlag==true)
    {
       if (lin_angs.data>330 || lin_angs.data<25)
       {
            angDir.data=160;
            //pubLight.publish("diL");
            ROS_INFO("Passing left");
            lights.data="le";
       }   

       else if ( 85<lin_angs.data && lin_angs.data<220)
       {
            if (lin_dists.data>0.50)
            {
              lights.data="diL";
              passFlag=false;
              ROS_INFO("DONE passing!");
            }
            else 
            {
              lights.data="ri";
              angDir.data=37;
              ROS_INFO("Passing right");
            }
       }
       else if( 20<lin_angs.data && lin_angs.data<85)
       {
             lights.data="diL";
             angDir.data=lineFollowSteering.data;
       }
       else
       {
          ROS_INFO("B");
       }
       vel.data=-500;
    }
/////////////////////////////Lane following routine/////////////////////////////////////
    else if (stopFlag==false)
    {
      if(lin_dist.data>90)
      {
        lights.data="diL";
        angDir.data=lineFollowSteering.data;
        //gvel=lineFollowSpeed.data;
        vel.data=velCatch(vel,lineFollowSpeed);
        ROS_INFO("a: Lane Following");
      if(16>lin_angs.data || lin_angs.data>344)
        {
          angDir.data=lineFollowSteering.data;
          vel.data= controlDist(lin_dists);
          ROS_INFO("a: Obstacle Following");
          if(vel.data==0)
          {
             cnt++;
             ROS_INFO("cnt:%i",cnt);
          }
          else 
          {
               cnt=0; 
          }
          if(cnt>150)
          {  
            cnt=0;
            passFlag=true;    
          }
        }
        else 
        {  
             angDir.data=lineFollowSteering.data;
             //gvel=lineFollowSpeed.data;
             vel.data=velCatch(vel,lineFollowSpeed);
             ROS_INFO("a:Line Following");
        }
      }
      else if(15<lin_dist.data && lin_dist.data<90)
      {
          angDir.data=lineFollowSteering.data;
          vel.data=-250;
          lights.data="stop";
          ROS_INFO("a: Stopping");
          ros::Duration(0.1).sleep();
      }
      else if(15>lin_dist.data)
      {
        vel.data=0;
        angDir.data=98-lin_ang.data;
        stopFlag= true;
        lights.data="diL";
        ROS_INFO("a. Stopped");
        ros::Duration(0.1).sleep();
      }
      else
      {
        ROS_INFO("Not defined...");
      }
    }

 else if (stopFlag==true)
  {
    ROS_INFO("cnt:%i",cnt);
    if((lin_dists.data<0.2 || (30<lin_angs.data && lin_angs.data<210) || (298<lin_angs.data && lin_angs.data<330)) && cnt>5)
    {
      if (lin_dist.data<12)
      {
           stopFlag=false;
           vel.data=-300;
           cnt=0;
           lights.data="diL";
           ROS_INFO("b:Crossing DONE!");
           ros::Duration(0.8).sleep();
           //lin_dist.data=100;
           //angDir.data=98;
      }
      else
      {
         angDir.data=98-lin_ang.data;
         vel.data=-350;
         lights.data="ta";
         ROS_INFO("b:Crossing");
      }
    }
    else 
    {
        if (15>lin_dist.data)
        {
           vel.data=-100;
           pubVel.publish(vel);
           ros::Duration(0.1).sleep();
           vel.data=0;
           pubVel.publish(vel);
           lights.data="stop";
           ROS_INFO("b: Getting closer");
           
        }
        else 
        {
            vel.data=0;
            angDir.data=98;
            cnt++;
            lights.data="diL";
            ROS_INFO("b. Waiting");
         }
      ros::Duration(0.1).sleep();
    }
  }
    pubLight.publish(lights);
    pubDir.publish(angDir);
    pubVel.publish(vel);
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding= sensor_msgs::image_encodings::MONO8; //BGR8
    out_msg.image= image; 
    image_pub_.publish(out_msg.toImageMsg());

int e2=cv::getTickCount();
float t=(e2-e1)/cv::getTickFrequency();
ROS_INFO("frame time: %f------------------------------------",t);        
ROS_INFO("scan_dist:%f     |       scan_ang:%d",lin_dists.data,lin_angs.data);
ROS_INFO("line_dist:%d     |       ang_Dir:%d      | vel:%d",lin_dist.data,angDir.data,vel.data); 
ROS_INFO("----------------------------------------block end");

 }
};
int main(int argc, char** argv) 
{
ros::init(argc, argv, "Master");
ROS_INFO("my_node running...");
Master ic;
//int cnt=0;
//bool stopFlag=false;
//bool passflag=false;
angDir.data=98;
vel.data=0;
lineFollowSteering.data=100;
lineFollowSpeed.data=0;
lin_dists.data=0.1;
lin_angs.data=220;
lin_dist.data=100;
lin_ang.data=180;
lights.data="dil";
ros::spin();
return 0;
}
