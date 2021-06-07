#include <ros.h>
#include <CDS5500.h>
#include "youbot_gripper/grip_service.h"
#include <std_msgs/Float64.h>

#define CLOSED_POS_S1 930
#define CLOSED_POS_S2 90
#define ROTATION_LIMIT 340
#define POS_ERROR 30   //assuming the data might have uncertainty
#define LOAD_ERROR 50  //assuming the data might have uncertainty
#define GRASP_LOAD 1000

/*http://image.dfrobot.com/image/data/SER0026/cds55xx-robot-servo-datasheet.pdf 
330 degrees and 1023 positions. 1 position is equal to 0.32258064516 degrees 
rotation limit(positions)is 340 or 109.677419354 degrees or 1.9024088846691 radians
*/


CDS5500 SERVO;
ros::NodeHandle  nh;

bool add(youbot_gripper::grip_service::Request  &req,
         youbot_gripper::grip_service::Response &res);

ros::ServiceServer<youbot_gripper::grip_service::Request, youbot_gripper::grip_service::Response> server("youbot_gripper", &add);

std_msgs::Float64 twin_msg_2; //0 :  -1.6
ros::Publisher digital_twin_2("gripper/gripper_controller_1/command", &twin_msg_2);


std_msgs::Float64 twin_msg_1; //0 : 1.6
ros::Publisher digital_twin_1("gripper/gripper_controller/command",   &twin_msg_1);


void setup()
{ 
           Serial1.begin(1000000);
           nh.initNode();
           nh.advertiseService(server);
           nh.advertise(digital_twin_2);
           nh.advertise(digital_twin_1);
           while(!nh.connected()) 
               nh.spinOnce();
           nh.loginfo("Gripper ready.");
           SERVO.WritePos(1,  CLOSED_POS_S1, 0x01FF);
           SERVO.WritePos(12, CLOSED_POS_S2, 0x01FF);
}

void loop()
{
           nh.spinOnce();
           delay(1);
}


bool add(youbot_gripper::grip_service::Request  &req, youbot_gripper::grip_service::Response &res)
{
   res.result = false;
   nh.loginfo("Got a new request."); 
   if(data_check(req.pos) && safety_check())
   { 
    switch(static_cast<int>(req.pos / 1000))
     {    case 0:
                 int lastServoLoad = SERVO.Get_Load(1);
                 twin_msg_1.data =  (req.pos*0.016);
                 twin_msg_2.data = -(req.pos*0.016);
                 digital_twin_1.publish( &twin_msg_1 );
                 digital_twin_2.publish( &twin_msg_2 );
                 SERVO.WritePos(1, (CLOSED_POS_S1 - map(req.pos, 0, 100, 0, ROTATION_LIMIT)), 0x01FF);
                 SERVO.WritePos(12,(CLOSED_POS_S2 + map(req.pos, 0, 100, 0, ROTATION_LIMIT)), 0x01FF);
                 
                 //checking whether object was grasped or not
                 while(abs(SERVO.Get_Pos(1) - (CLOSED_POS_S1 - map(req.pos, 0, 100, 0, ROTATION_LIMIT))) > POS_ERROR)
                     {
                      delay(500);
                      if((abs(lastServoLoad - SERVO.Get_Load(1)) <= LOAD_ERROR) && (SERVO.Get_Load(1) > GRASP_LOAD))
                          break;
                      lastServoLoad = SERVO.Get_Load(1);                            
                     }
                  delay(200);
                  if(SERVO.Get_Load(1)>GRASP_LOAD)
                    {
                     nh.loginfo("Object has been grasped.");                        
                     res.result = true;
                    }                     
                 break;
          case 1:
                 SERVO.WritePos(1, (CLOSED_POS_S1 - (req.pos - 1000)), 0x01FF);
                 twin_msg_1.data = (req.pos*0.016);
                 digital_twin_1.publish( &twin_msg_1 );
                 res.result = true;                     
                 break;
          case 2:
                 SERVO.WritePos(12, (CLOSED_POS_S2 + (req.pos - 2000)), 0x01FF);
                 twin_msg_2.data = -(req.pos*0.016);
                 digital_twin_2.publish( &twin_msg_2 );
                 res.result = true;   
                 break;
          default:
                 nh.loginfo("2");
                 nh.logerror("Invalid command.");
                 res.result = false;
                 break;
     }
  } else 
      {
         nh.logerror("Invalid command: the number should be in range(0:100; 1000:1380; 2000;2380.");
         res.result = false;
         return false;
      } 
  nh.loginfo("Sending back response.");
  return true;
}

bool safety_check()
  { /*
      checking actual voltage and temperature
      More info about limits: http://image.dfrobot.com/image/data/SER0026/cds55xx-robot-servo-datasheet.pdf
    */
    if(SERVO.Get_Voltage(1) <= 6.6 || SERVO.Get_Voltage(1) >= 16)
      {
        nh.logerror("Aborted. Unsafe voltage");
        return false;
      }
    if(SERVO.Get_Temp(1) <= -20 || SERVO.Get_Temp(1) >= 80)
      {
        nh.logerror("Aborted. Unsafe temperature");
        return false;
      }
    if(SERVO.Get_Pos(1) < 525 || SERVO.Get_Pos(1) > 947)
      {
        nh.logerror("Aborted. Unsafe position of the fitst servo");
        return false;
      }  
    return true;   
  }


bool data_check(int number) //checking the command
  {
    return ((number >= 0  && number <= 100) || (number >= 1000 && number <= 1340) || (number >= 2000 && number <= 2340));

  }
