#include <PS2X_lib.h>  //for v1.6
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>

#define PS2_DAT        A9  //14    
#define PS2_CMD        A10  //15
#define PS2_SEL        A11  //16
#define PS2_CLK        A12  //17


#define pressures   false
#define rumble      false

PS2X ps2x; // create PS2 Controller Class
ros::NodeHandle node_handle;

int error = 0;
byte type = 0;
byte vibrate = 0;

// parameters for reading the joystick:
int range = 127;               // output range of X or Y movement
int responseDelay = 5;        // response delay of the mouse, in ms
int threshold = 2;      // resting threshold
int center = 127;         // resting position value

sensor_msgs::JointState msg;
std_msgs::Int8 msg_;
ros::Publisher pub("joystick", &msg);

char* id = "/joint";
char *a[] = {"FL", "FR", "BR", "BL","BA","BC","BV","START","SELECT","CIRCLE","TRIAGNLE","SQUARE","CROSS","Z2"};
float pos[14];

void setup(){
 
  node_handle.getHardware()->setBaud(57600);
  
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  node_handle.initNode();
  node_handle.advertise(pub);

  
  msg.header.frame_id = id;
  msg.name_length = 14;
  msg.position_length = 14;
  msg.name= a;
  msg.position = pos;
}

int x_ = 0;
int y_ = 0;
int z_ = 0;
int lx_ = 0;
int ly_ = 0;
int rx_ = 0;
int ry_ = 0;
bool start_ = 0;
int select_ = 0;
int circle_ = 0;
int triangle_ =0;
int square_ = 0;
int cross_ = 0;
int z2_ = 0;

void loop() {

  if(error == 1) //skip loop if no controller found
    return; 
  
   ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_PAD_LEFT)){
      // Serial.print("LEFT held this hard: ");
      // Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);                  //-x
      x_ = -1;
    }
    else if(ps2x.Button(PSB_PAD_RIGHT)){
      // Serial.print("Right held this hard: ");
      // Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);                 //+x
      x_ = 1;
    }
    else{
      x_ = 0;
    }
    
    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      // Serial.print("Up held this hard: ");
      // Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);                    //-y
      y_ = -1;
    }
    else if(ps2x.Button(PSB_PAD_DOWN)){
      // Serial.print("DOWN held this hard: ");
      // Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);                  //+y
      y_ = +1;
    }  
    else{
      y_ = 0;
    }
     
    if(ps2x.Button(PSB_L2)){
      // Serial.println("L2 pressed");                                   //-z
      z_ = -1;
    }
    else if(ps2x.Button(PSB_L1)){
        // Serial.println("L1 pressed");                                   //+z
        z_ = +1;
    }
    else{
        z_ = 0;
    }

    if(ps2x.Button(PSB_R2)){
      // Serial.println("L2 pressed");                                   //-z
      z2_ = -1;
    }
    else if(ps2x.Button(PSB_R1)){
        // Serial.println("L1 pressed");                                   //+z
        z2_ = +1;
    }
    else{
        z2_ = 0;
    }

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_START)){         //will be TRUE as long as button is pressed
        Serial.println("Start is being held");
        start_ = !start_;
      }
      if(ps2x.Button(PSB_SELECT)){
        Serial.println("Select is being held"); 
        select_ += 1;
      }
      if(ps2x.Button(PSB_CIRCLE)){               //will be TRUE if button was JUST pressed
        Serial.println("Circle just pressed");
        circle_ = 1;
      }
      else if(ps2x.ButtonReleased(PSB_CIRCLE))
        circle_ = 0;
        
      if(ps2x.Button(PSB_CROSS)){               //will be TRUE if button was JUST pressed OR released
        Serial.println("X just changed");
        cross_ = 1;
      }
      else if(ps2x.ButtonReleased(PSB_CROSS))
        cross_ = 0;
        
      if(ps2x.Button(PSB_SQUARE)){              //will be TRUE if button was JUST released
        Serial.println("Square just released");
        square_ = 1;
      }
      else if(ps2x.ButtonReleased(PSB_SQUARE))
        square_ = 0;     
        
      if(ps2x.Button(PSB_TRIANGLE)){
        Serial.println("Triangle pressed");
        triangle_ = 1; 
      }  
      else if(ps2x.ButtonReleased(PSB_TRIANGLE))
        triangle_ = 0;   
    }

    lx_ = readAxis(PSS_LX);
    ly_ = readAxis(PSS_LY); 
    rx_ = readAxis(PSS_RX);
    ry_ = readAxis(PSS_RY); 

    // if(abs(rx_)>0)
    //     Serial.println(rx_, DEC);
    // if(abs(ry_)>0)
    //     Serial.println(ry_, DEC);

//     Serial.println(x_, DEC); 

  msg.position[0] = x_; 
  msg.position[1] = y_; 
  msg.position[2] = z_; 
  msg.position[3] = lx_; 
  msg.position[4] = ly_;
  msg.position[5] = rx_; 
  msg.position[6] = ry_;
  msg.position[7] = start_;
  msg.position[8] = select_;
  msg.position[9] = circle_;
  msg.position[10] = triangle_;
  msg.position[11] = square_;
  msg.position[12] = cross_;
  msg.position[13] = z2_;


  // msg.header.stamp = ros::Time::now();
  
  pub.publish( &msg );

  node_handle.spinOnce();
  delay(25);  
}

int readAxis(int thisAxis) { 
  // read the analog input:
  int reading = ps2x.Analog(thisAxis);

  // map the reading from the analog input range to the output range:
  // reading = map(reading, 0, 255, 0, range);

  // if the output reading is outside from the
  // rest position threshold,  use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  } 

  // return the distance for this axis:
  return distance;
}
