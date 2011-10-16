#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/*
Camera controls using libwebcam library and the uvcdynctrl interface

http://forums.quickcamteam.net/showthread.php?tid=212

Usage: uvcdynctrl [OPTIONS]... [VALUES]...

-h, --help Print help and exit
-V, --version Print version and exit
-l, --list List available cameras
-i, --import=filename Import dynamic controls from an XML file
-v, --verbose Enable verbose output (default=off)
-d, --device=devicename Specify the device to use (default=`video0')
-c, --clist List available controls
-g, --get=control Retrieve the current control value
-s, --set=control Set a new control value
(For negative values: -s 'My Control' -- -42)
-f, --formats List available frame formats

To get the present values:
uvcdynctrl -cv

controls:

# uvcdynctrl -s "Pan/tilt Reset" -- 1
reset pan
# uvcdynctrl -s "Pan/tilt Reset" -- 2
resets tilt
# uvcdynctrl -s "Pan/tilt Reset" -- 3
resets pan/tilt

# uvcdynctrl -s "Pan (relative)" -- -640
clockwise from the top
$ uvcdynctrl -s "Pan (relative)" -- 640
anticlockwise from the top
The range is 70º; 640 is 10 degress

# uvcdynctrl -s "Tilt (relative)" -- -320
tilts up
# uvcdynctrl -s "Tilt (relative)" -- 320
tilts down
The range is 30º and 320 is 5 degrees

Bits 0 to 15 control pan, bits 16 to 31 control tilt.
The unit of the pan/tilt values is 1/64th of a degree and the resolution is 1 degree.

1 point is 1/64degrees so to move x degreses
Xdegrees / 0.01562
10 degrees= 10/0.01562= 640 points

# uvcdynctrl -s "Focus" -- 0 infinity focus
# uvcdynctrl -s "Focus" -- 255 macro focus

Bits 0 to 7 allow selection of the desired lens position.
There are no physical units, instead, the focus range is spread over 256 logical units with 0 representing infinity focus and 255 being macro focus.

# uvcdynctrl -s "Brightness" -- 1
# uvcdynctrl -s "Brightness" -- 255

# uvcdynctrl -s "Contrast" -- 1
# uvcdynctrl -s "Contrast" -- 255

# uvcdynctrl -s "Saturation" -- 1
# uvcdynctrl -s "Saturation" -- 255

#uvcdynctrl -s "Exposure, Auto" -- 0 disable
#uvcdynctrl -s "Exposure, Auto" -- 3 enable

# uvcdynctrl -s "Gain" -- 1
# uvcdynctrl -s "Gain" -- 255

The auto exposure control needs to be disabled before the gain works. Otherwise, the auto exposure algorithm will take control over both, the exposure time, and the gain (Martin)
There is no zoom support in the camera itself. Gain, on the other hand, is how much the sensor increases the input signal of the individual pixels.(Martin)

# uvcdynctrl -s "Backlight Compensation" -- 0
# uvcdynctrl -s "Backlight Compensation" -- 1
# uvcdynctrl -s "Backlight Compensation" -- 2


# uvcdynctrl -s "Power Line Frequency" -- 0
# uvcdynctrl -s "Power Line Frequency" -- 1
# uvcdynctrl -s "Power Line Frequency" -- 2

# uvcdynctrl -s "White Balance Temperature" -- x
I do not see anything neither errors The same with
Auto Priority
White Balance Temperature, Auto

# uvcdynctrl -s "LED1 Mode" -- 0 LED off. The LED is never illuminated, whether or not the device is streaming video.
# uvcdynctrl -s "LED1 Mode" -- 1 LED on. The LED is always illuminated, whether or not the device is streaming video.
# uvcdynctrl -s "LED1 Mode" -- 2 LED blinking. The LED blinks, whether or not the device is streaming video.
# uvcdynctrl -s "LED1 Mode" -- 3 LED Auto. The LED is in control of the device. Typically this means that means that is is illuminated when streaming video and off when not streaming video. 

*/

#define MIN_PAN -4480
#define MAX_PAN 4480
#define DELTA_PAN 640	// corresponds to 10 degrees left, range is +- 70 degrees

#define MIN_TILT -1920
#define MAX_TILT 1920
#define DELTA_TILT 320  // corresponds to 5 degrees down, range is +- 30 degrees

#define CAMERA_CHANGE_STRING "cam"
#define CAMERA_CHANGE_STRING_CAP "Cam"

std::string baseMsg, cmdMsg, currentVideoDeviceMsg;
std::string pan_tilt_camera_device, lower_camera_device;	// ros parameters that define which cameras to use
bool usingFirstVideoDevice;
int numSteps;


void pan(int panDelta)
{
   std::stringstream distance;
   cmdMsg = currentVideoDeviceMsg;
   distance << panDelta;
   cmdMsg.append(" -s \"Pan (relative)\" -- ");
   cmdMsg.append(distance.str());
   system(cmdMsg.c_str());
   ROS_INFO("%s", cmdMsg.c_str());
 
   
   /*
   std::stringstream distance;
   std_msgs::String msg = "uvcdynctrl -s \"Pan (relative)\" -- ";
   distance << msg << panDelta;
   msg.data = distance.str();
   system(msg.data.c_str());
   ROS_INFO("%s", msg.data.c_str());
   */
}


void tilt(int tiltDelta)
{
   std::stringstream distance;
   cmdMsg = currentVideoDeviceMsg;
   distance << tiltDelta;
   cmdMsg.append(" -s \"Tilt (relative)\" -- ");
   cmdMsg.append(distance.str());
   system(cmdMsg.c_str()); 
   ROS_INFO("%s", cmdMsg.c_str());
}

void skypeCallback( const std_msgs::String& msgSkype)
{
    std_msgs::String ReceivedCommands = msgSkype;
    numSteps = strlen( (const char* ) msgSkype.data.c_str());
    std::string skypeString = msgSkype.data.c_str();
    ROS_INFO("%s", skypeString.c_str());
    if ( numSteps > 10 ) numSteps = 2;  // probably a mistake, just move a little
    else if ( numSteps > 5 ) numSteps = 5;
    
    if (skypeString.compare(CAMERA_CHANGE_STRING) == 0 || skypeString.compare(CAMERA_CHANGE_STRING_CAP) == 0)  // swap cameras, both video and pan tilt
    {
    	 currentVideoDeviceMsg = baseMsg;
    	 if (usingFirstVideoDevice)
    	 {
    	 	currentVideoDeviceMsg.append(lower_camera_device);
    	 	usingFirstVideoDevice = false;
    	 }
    	 else
    	 {
    	 	currentVideoDeviceMsg.append(pan_tilt_camera_device);
    	 	usingFirstVideoDevice = true; 
    	 } 
    	 ROS_INFO("%s", "pan tilt camera change");
    	 return;
    }  	
    	 	
    for (int i = 1; i < numSteps; i++) if ( (msgSkype.data[i] != msgSkype.data[0]) && msgSkype.data[i] != msgSkype.data[0] + 32 ) return; 
          // if string is not all identical characters, allowing for first character to be a capital, return    
          
    if (!usingFirstVideoDevice) 
    {
    	ROS_INFO("%s", "pan tilt command received when not looking through a pan tilt camera!");
    	return;
    }
    
    char cmd = msgSkype.data[0];  
    switch(cmd)
    {
            
     case 'u':  // tilt up
     case 'U':
        tilt(-numSteps * DELTA_TILT);
        break;
  
      case 'n':  // tilt down
      case 'N':  // tilt down
        tilt(numSteps * DELTA_TILT); 
        break;
        
      case 'b':  // center tilt
      case 'B':
      	cmdMsg = currentVideoDeviceMsg;
      	cmdMsg.append(" -s \"Pan/tilt Reset\" -- 2");
      	system(cmdMsg.c_str());
      	break;
   
      case 'k': // pan right
      case 'K':
        pan(-numSteps * DELTA_PAN);
        break;
 
      case 'h': // pan left
      case 'H':
        pan(numSteps * DELTA_PAN);
        break;
        
      case 'g': // center pan
      case 'G':
      	cmdMsg = currentVideoDeviceMsg;
      	cmdMsg.append(" -s \"Pan/tilt Reset\" -- 1");
      	system(cmdMsg.c_str());
      	break;
  	       
      case 'j': //center all
      case 'J':
      	cmdMsg = currentVideoDeviceMsg;
      	cmdMsg.append(" -s \"Pan/tilt Reset\" -- 3");
      	system(cmdMsg.c_str());
        break;
    
      case 'm':  //max down tilt 
      case 'M':  
        tilt(MAX_TILT * 2);  // could be as high as max tilt up, so need to go down twice the range.
        break;
      	
      /*  we no longer have a lower pan/tilt
      
      case '0': // swap video device, using different command than used to swap both video and pan tilt, this is pan tilt only
      	currentVideoDeviceMsg = baseMsg;
      	if (usingFirstVideoDevice)
      	{
      		currentVideoDeviceMsg.append(SECOND_VIDEO_DEVICE);
    		usingFirstVideoDevice = false;
    	}
    	else
    	{
    		currentVideoDeviceMsg.append(FIRST_VIDEO_DEVICE);
    		usingFirstVideoDevice = true;
    	}
    	break;
    	
    	case '1':  // use first video device
      		currentVideoDeviceMsg = baseMsg;
      		currentVideoDeviceMsg.append(FIRST_VIDEO_DEVICE);
    		usingFirstVideoDevice = true;
      		break;
      	
      	case '2': // use second video device
      		currentVideoDeviceMsg = baseMsg;
      		currentVideoDeviceMsg.append(SECOND_VIDEO_DEVICE);
    		usingFirstVideoDevice = false;
      		break;
      		
     	*/
      default:  // unknown command
      	break;
    }

}
 
int main(int argc, char **argv)
{
 ros::init(argc, argv, "cameraControl");
 ros::NodeHandle nh; 
 ros::Subscriber subscriberSkype = nh.subscribe("SkypeChat", 1000, skypeCallback);  
 ros::Rate loop_rate(50);

 //look for the parameter in the parameter server
 //if exists assign the current value
 //otherwise assign the default value ("defualt" in this case)
 //recall that this line won't add the parameter on the parameter server
 nh.param<std::string>("pan_tilt_camera", pan_tilt_camera_device, "/dev/video1");
 nh.param<std::string>("lower_camera", lower_camera_device, "/dev/video0");

 baseMsg = "uvcdynctrl -d ";
 currentVideoDeviceMsg = baseMsg;
 currentVideoDeviceMsg.append(pan_tilt_camera_device);
 usingFirstVideoDevice = true;
 
 while (ros::ok())
   {
		ros::spinOnce();
   		loop_rate.sleep();
   }
}
