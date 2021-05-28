#include <ros.h>
#include <std_msgs/String.h>
#include <CDS5500.h>
#include "youbot_gripper/grip_service.h"

CDS5500 SERVO;
ros::NodeHandle  nh;
std_msgs::String str_msg;

bool add(youbot_gripper::grip_service::Request  &req,
         youbot_gripper::grip_service::Response &res);

ros::ServiceServer<youbot_gripper::grip_service::Request, youbot_gripper::grip_service::Response> server("youbot_gripper", &add);

void setup()
{ 
           Serial1.begin(1000000);
           nh.initNode();
           nh.advertiseService(server);
           while(!nh.connected()) 
               nh.spinOnce();
           nh.loginfo("Gripper ready.");
           SERVO.WritePos(1, 925, 0x01FF);
           SERVO.WritePos(12,90, 0x01FF);
}

void loop()
{
           nh.spinOnce();
           delay(1);
}

bool add(youbot_gripper::grip_service::Request  &req,
         youbot_gripper::grip_service::Response &res)
{
 res.result=false;
 nh.loginfo("Got new request."); 
 if(data_check(req.pos) && safety_check())
{
  switch(static_cast<int>(req.pos/1000))
             {    case 0:
                         SERVO.WritePos(1, (930-map(req.pos,0,100,0,380)), 0x01FF);
                         SERVO.WritePos(12,(90 +map(req.pos,0,100,0,380)), 0x01FF);
                         int last_servo_load=SERVO.Get_Load(1);
                         int pos_error=30;  //assuming the data might have uncertainty
                         int load_error=50; //assuming the data might have uncertainty
                         int grasping_load=1000; 
                         //checking whether object was grasped or not
                         while(abs(SERVO.Get_Pos(1)-(930-map(req.pos,0,100,0,380)))>pos_error)
                             {
                              delay(500);
                              if((abs(last_servo_load-SERVO.Get_Load(1))<=load_error) && (SERVO.Get_Load(1)>grasping_load))
                                  break;
                               last_servo_load=SERVO.Get_Load(1);                            
                             }
                          delay(200);
                          if(SERVO.Get_Load(1)>grasping_load)
                            {
                             nh.loginfo("Object grasped.");                        
                             res.result = true;
                            }
                         
                         break;
                  case 1:
                         SERVO.WritePos(1,(930-(req.pos-1000)), 0x01FF);
                         res.result = true;                    
                         break;
                  case 2:
                         SERVO.WritePos(12,(90+(req.pos-2000)), 0x01FF);
                         res.result = true;
                         break;
                  default:
                         nh.logerror("Invalid command");
                         res.result = false;
          
             }
  }
  else 
      res.result = false;
      
  nh.loginfo("Sending back response.");

  return true;
}

bool safety_check()
  { /*
      checking actual voltage, position and temperature
      More info about limits: http://image.dfrobot.com/image/data/SER0026/cds55xx-robot-servo-datasheet.pdf
    */
    if(SERVO.Get_Voltage(1)<=6.6 || SERVO.Get_Voltage(1)>=16)
      {
        nh.logerror("Aborted. Unsafe voltage");
        return false;
      }
    if(SERVO.Get_Temp(1)<=-20 || SERVO.Get_Temp(1)>=80)
      {
        nh.logerror("Aborted. Unsafe temperature");
        return false;
      }
    if(SERVO.Get_Pos(1)<525 || SERVO.Get_Pos(1)>947)
      {
        nh.logerror("Aborted. Unsafe position of the fitst servo");
        return false;
      }
      
      return true;
     
  }


bool data_check(int number) //checking the command
  {
    if((number>=0) || (number<=100)
    || (number>=1000 && number<=1380)
    || (number>=2000 && number<=2380))
        return true;
    else
        {
        nh.logerror("Invalid command: the number should be in range(0:100; 1000:1380; 2000;2380.");
        return false;
        }
  }
