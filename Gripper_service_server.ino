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
           //nh.loginfo("Checking Voltage.");
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
 if(data_check(req.pos))
  switch(req.pos) 
  {
    case 0:
                 SERVO.WritePos(1,550, 0x01FF);
                 SERVO.WritePos(12,470, 0x01FF);
                 res.result = true;
                 break;
    case 1: 
                 SERVO.WritePos(1,925, 0x01FF);
                 SERVO.WritePos(12,90, 0x01FF);
                 res.result = true;
                 break;
    default:
            switch(static_cast<int>(req.pos/1000))
             {    case 0:
                         SERVO.WritePos(1,(930-(req.pos-1000)), 0x01FF);
                         SERVO.WritePos(12,(90+(req.pos-2000)), 0x01FF);
                         res.result = true;
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
                         nh.logerror("Invalid command 1.");
                         res.result = false;
          
             }
      
  }
  else 
    { 
      nh.logerror("Invalid command 3.");
      res.result = false;
    }
  nh.loginfo("Sending back response.");

  return true;
}

bool safety_check(int number) //checking if the movement of one finger is safe for another finger
  {
    bool check=false;
    switch(static_cast<int>(number/1000))
        {
          case 1: //if i want to move first servo i have to check the second servo for its position to avoid the collision
                int current_pos2=SERVO.Get_Pos(12); //getting data from position sensor of the servo with the 12th ID
                if((current_pos2)>=80 && (current_pos2)<=480)  //borders are 90 and 470. 10 is a measurement error
                    {
                      nh.loginfo("assigned TRUE 1");
                      check=true;
                    }
                else 
                   {
                    nh.logerror("Can't move the first servo because it conflicts with the the second servo's position or the second servo is on the wrong angle. Please, restart gripper."); 
                    check=false;
                   }
                break;

          case 2:
                int current_pos1=SERVO.Get_Pos(1); //getting data from position sensor of the servo with the 1st ID
                if(((current_pos1)>=540) && (current_pos1)<=945)  //borders are 930 and 550. 10 is a measurement error
                    {
                      nh.loginfo("assigned TRUE 2 ");
                      check=true;
                    }
                else 
                    {
                      nh.logerror("Can't move the second servo because it conflicts with the first servo's position or the first servo is on the wrong angle. Please, restart gripper."); 
                      check=false;
                    }
                break;

          default:
                nh.logerror("Invalid command 1.");
                check=false;
                 
        }
      nh.loginfo("enter 3");
      return check;
  }

bool data_check(int number) //checking the command
  {
    if((number==0) || (number==1)
    || (number>=1000 && number<=1380) || (number>=2000 && number<=2380))
        return true;
    else
        {
        nh.logerror("Invalid command 2.");
        return false;
        }
  }
