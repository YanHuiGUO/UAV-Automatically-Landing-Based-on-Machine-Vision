#ifndef _MARVLINK_H
#define _MARVLINK_H

#define MAV_MESSAGE_LEN 300 //MAVLINK协议CRC字典长度

#include "include.h"
extern const int MAV_GCS_sysid;
extern const int MAV_GCS_WP_comid;	
extern const int MAV_APM_sysid ;
extern const int MAV_APM_comid ;
extern const UINT_8 message_info[MAV_MESSAGE_LEN];

//basemode
enum MAV_BASE_MODE
{
   CUSTOM_MODE_ENABLED = 1,
   TEST_ENABLED = 2,
   AUTO_ENABLED = 4,
   GUIDED_ENABLED = 8,
   STABILIZE_ENABLED = 16,
   HIL_ENABLED = 32,
   MANUAL_INPUT_ENABLED = 64,
   SAFETY_ARMED = 128,
   
};
enum MAV_CUSTOME_MODE
{
   Stabilize = 0,//自稳
   Acro = 1,//特技
   AltHold = 2,//定高
   Auto = 3,//自动
   Guided = 4,//引导
   Loiter = 5,//留待
   RTL = 6,//返航
   Circle = 7,//绕圈
   Land = 9,//降落
	 Drift = 11,//漂移
	 Sport = 13,//运动
	 Flip = 14, //翻转
	 AutoTune = 15,//自动调参
	 PosHold = 16,//定点
	 Brake = 17 //暂停
};


///<summary> Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. </summary>
enum MAV_CMD
{
			///<summary> Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  </summary>
        WAYPOINT=16, 
    	///<summary> Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  </summary>
        LOITER_UNLIM=17, 
    	///<summary> Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  </summary>
        LOITER_TURNS=18, 
    	///<summary> Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  </summary>
        LOITER_TIME=19, 
    	///<summary> Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        RETURN_TO_LAUNCH=20, 
    	///<summary> Land at location |Abort Alt| Empty| Empty| Desired yaw angle| Latitude| Longitude| Altitude|  </summary>
        LAND=21, 
    	///<summary> Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  </summary>
        TAKEOFF=22, 
    	///<summary> Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate [ms^-1]| Desired yaw angle [rad]| Y-axis position [m]| X-axis position [m]| Z-axis / ground level position [m]|  </summary>
        LAND_LOCAL=23, 
    	///<summary> Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]| Empty| Takeoff ascend rate [ms^-1]| Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position [m]| X-axis position [m]| Z-axis position [m]|  </summary>
        TAKEOFF_LOCAL=24, 
    	///<summary> Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  </summary>
        FOLLOW=25, 
    	///<summary> Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. | Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  </summary>
        CONTINUE_AND_CHANGE_ALT=30, 
    	///<summary> Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.  |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location| Latitude| Longitude| Altitude|  </summary>
        LOITER_TO_ALT=31, 
    	///<summary> Being following a target |System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode| RESERVED| RESERVED| altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home| altitude| RESERVED| TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout|  </summary>
        DO_FOLLOW=32, 
    	///<summary> Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target (m)| X offset from target (m)| Y offset from target (m)|  </summary>
        DO_FOLLOW_REPOSITION=33, 
    	///<summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  </summary>
        ROI=80, 
    	///<summary> Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  </summary>
        PATHPLANNING=81, 
    	///<summary> Navigate to MISSION using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  </summary>
        SPLINE_WAYPOINT=82, 
    	///<summary> Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |altitude (m)| descent speed (m/s)| Wiggle Time (s)| Empty| Empty| Empty| Empty|  </summary>
        ALTITUDE_WAIT=83, 
    	///<summary> Takeoff from ground using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees| Latitude| Longitude| Altitude|  </summary>
        VTOL_TAKEOFF=84, 
    	///<summary> Land using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees| Latitude| Longitude| Altitude|  </summary>
        VTOL_LAND=85, 
    	///<summary> hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        GUIDED_ENABLE=92, 
    	///<summary> Delay the next navigation command a number of seconds or until a specified time |Delay in seconds (decimal, -1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC)| Empty| Empty| Empty|  </summary>
        DELAY=93, 
    	///<summary> NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        LAST=95, 
    	///<summary> Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        CONDITION_DELAY=112, 
    	///<summary> Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  </summary>
        CONDITION_CHANGE_ALT=113, 
    	///<summary> Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        CONDITION_DISTANCE=114, 
    	///<summary> Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  </summary>
        CONDITION_YAW=115, 
    	///<summary> NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        CONDITION_LAST=159, 
    	///<summary> Set system mode. |Mode, as defined by ENUM MAV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  </summary>
        DO_SET_MODE=176, 
    	///<summary> Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_JUMP=177, 
    	///<summary> Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| absolute or relative [0,1]| Empty| Empty| Empty|  </summary>
        DO_CHANGE_SPEED=178, 
    	///<summary> Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  </summary>
        DO_SET_HOME=179, 
    	///<summary> Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_SET_PARAMETER=180, 
    	///<summary> Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_SET_RELAY=181, 
    	///<summary> Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  </summary>
        DO_REPEAT_RELAY=182, 
    	///<summary> Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_SET_SERVO=183, 
    	///<summary> Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  </summary>
        DO_REPEAT_SERVO=184, 
    	///<summary> Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_FLIGHTTERMINATION=185, 
    	///<summary> Change altitude set point. |Altitude in meters| Mav frame of new altitude (see MAV_FRAME)| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_CHANGE_ALTITUDE=186, 
    	///<summary> Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  </summary>
        DO_LAND_START=189, 
    	///<summary> Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_RALLY_LAND=190, 
    	///<summary> Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_GO_AROUND=191, 
    	///<summary> Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.| Reserved| Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  </summary>
        DO_REPOSITION=192, 
    	///<summary> If in a GPS controlled position mode, hold the current position or continue. |0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
        DO_PAUSE_CONTINUE=193, 
    	///<summary> Set moving direction to forward or reverse. |Direction (0=Forward, 1=Reverse)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_SET_REVERSE=194, 
    	///<summary> Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  </summary>
        DO_CONTROL_VIDEO=200, 
    	///<summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  </summary>
        DO_SET_ROI=201, 
    	///<summary> Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  </summary>
        DO_DIGICAM_CONFIGURE=202, 
    	///<summary> Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  </summary>
        DO_DIGICAM_CONTROL=203, 
    	///<summary> Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  </summary>
        DO_MOUNT_CONFIGURE=204, 
    	///<summary> Mission command to control a camera or antenna mount |pitch or lat in degrees, depending on mount mode.| roll or lon in degrees depending on mount mode| yaw or alt (in meters) depending on mount mode| reserved| reserved| reserved| MAV_MOUNT_MODE enum value|  </summary>
        DO_MOUNT_CONTROL=205, 
    	///<summary> Mission command to set CAM_TRIGG_DIST for this flight |Camera trigger distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_SET_CAM_TRIGG_DIST=206, 
    	///<summary> Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_FENCE_ENABLE=207, 
    	///<summary> Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_PARACHUTE=208, 
    	///<summary> Mission command to perform motor test |motor sequence number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| Empty| Empty| Empty|  </summary>
        DO_MOTOR_TEST=209, 
    	///<summary> Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_INVERTED_FLIGHT=210, 
    	///<summary> Mission command to operate EPM gripper |gripper number (a number from 1 to max number of grippers on the vehicle)| gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum)| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_GRIPPER=211, 
    	///<summary> Enable/disable autotune |enable (1: enable, 0:disable)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_AUTOTUNE_ENABLE=212, 
    	///<summary> Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  </summary>
        DO_MOUNT_CONTROL_QUAT=220, 
    	///<summary> set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_GUIDED_MASTER=221, 
    	///<summary> set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  </summary>
        DO_GUIDED_LIMITS=222, 
    	///<summary> Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |0: Stop engine, 1:Start Engine| 0: Warm start, 1:Cold start. Controls use of choke where applicable| Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_ENGINE_CONTROL=223, 
    	///<summary> NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_LAST=240, 
    	///<summary> Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Accelerometer calibration: 0: no, 1: yes| Compass/Motor interference calibration: 0: no, 1: yes| Empty|  </summary>
        PREFLIGHT_CALIBRATION=241, 
    	///<summary> Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  </summary>
        PREFLIGHT_SET_SENSOR_OFFSETS=242, 
    	///<summary> Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
        PREFLIGHT_UAVCAN=243, 
    	///<summary> Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  </summary>
        PREFLIGHT_STORAGE=245, 
    	///<summary> Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| Reserved, send 0| Reserved, send 0| Reserved, send 0| Reserved, send 0| Reserved, send 0|  </summary>
        PREFLIGHT_REBOOT_SHUTDOWN=246, 
    	///<summary> Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  </summary>
        OVERRIDE_GOTO=252, 
    	///<summary> start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  </summary>
        MISSION_START=300, 
    	///<summary> Arms / Disarms a component |1 to arm, 0 to disarm|  </summary>
        COMPONENT_ARM_DISARM=400, 
    	///<summary> Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
        GET_HOME_POSITION=410, 
    	///<summary> Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  </summary>
        START_RX_PAIR=500, 
    	///<summary> Request the interval between messages for a particular MAVLink message ID |The MAVLink message ID|  </summary>
        GET_MESSAGE_INTERVAL=510, 
    	///<summary> Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM |The MAVLink message ID| The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.|  </summary>
        SET_MESSAGE_INTERVAL=511, 
    	///<summary> Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  </summary>
        REQUEST_AUTOPILOT_CAPABILITIES=520, 
    	///<summary> Start image capture sequence |Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  </summary>
        IMAGE_START_CAPTURE=2000, 
    	///<summary> Stop image capture sequence |Reserved| Reserved|  </summary>
        IMAGE_STOP_CAPTURE=2001, 
    	///<summary> Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start)| Shutter integration time (in ms)| Reserved|  </summary>
        DO_TRIGGER_CONTROL=2003, 
    	///<summary> Starts video capture |Camera ID (0 for all cameras), 1 for first, 2 for second, etc.| Frames per second| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  </summary>
        VIDEO_START_CAPTURE=2500, 
    	///<summary> Stop the current video capture |Reserved| Reserved|  </summary>
        VIDEO_STOP_CAPTURE=2501, 
    	///<summary> Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  </summary>
        PANORAMA_CREATE=2800, 
    	///<summary> Request VTOL transition |The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.|  </summary>
        DO_VTOL_TRANSITION=3000, 
    	///<summary> This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.                    | </summary>
        SET_GUIDED_SUBMODE_STANDARD=4000, 
    	///<summary> This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.                    |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Unscaled target latitude of center of circle in CIRCLE_MODE| Unscaled target longitude of center of circle in CIRCLE_MODE|  </summary>
        SET_GUIDED_SUBMODE_CIRCLE=4001, 
    	///<summary> Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  </summary>
        PAYLOAD_PREPARE_DEPLOY=30001, 
    	///<summary> Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
        PAYLOAD_CONTROL_DEPLOY=30002, 
    	///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        WAYPOINT_USER_1=31000, 
    	///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        WAYPOINT_USER_2=31001, 
    	///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        WAYPOINT_USER_3=31002, 
    	///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        WAYPOINT_USER_4=31003, 
    	///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        WAYPOINT_USER_5=31004, 
    	///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        SPATIAL_USER_1=31005, 
    	///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        SPATIAL_USER_2=31006, 
    	///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        SPATIAL_USER_3=31007, 
    	///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        SPATIAL_USER_4=31008, 
    	///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
        SPATIAL_USER_5=31009, 
    	///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        USER_1=31010, 
    	///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        USER_2=31011, 
    	///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        USER_3=31012, 
    	///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        USER_4=31013, 
    	///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
        USER_5=31014, 
    	///<summary> A system wide power-off event has been initiated. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        POWER_OFF_INITIATED=42000, 
    	///<summary> FLY button has been clicked. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        SOLO_BTN_FLY_CLICK=42001, 
    	///<summary> FLY button has been held for 1.5 seconds. |Takeoff altitude| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        SOLO_BTN_FLY_HOLD=42002, 
    	///<summary> PAUSE button has been clicked. |1 if Solo is in a shot mode, 0 otherwise| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        SOLO_BTN_PAUSE_CLICK=42003, 
    	///<summary> Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Automatically retry on failure (0=no retry, 1=retry).| Save without user input (0=require input, 1=autosave).| Delay (seconds)| Autoreboot (0=user reboot, 1=autoreboot)| Empty| Empty|  </summary>
        DO_START_MAG_CAL=42424, 
    	///<summary> Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_ACCEPT_MAG_CAL=42425, 
    	///<summary> Cancel a running magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_CANCEL_MAG_CAL=42426, 
    	///<summary> Command autopilot to get into factory test/diagnostic mode |0 means get out of test mode, 1 means get into test mode| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        SET_FACTORY_TEST_MODE=42427, 
    	///<summary> Reply with the version banner |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        DO_SEND_BANNER=42428, 
    	///<summary> Causes the gimbal to reset and boot as if it was just powered on |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        GIMBAL_RESET=42501, 
    	///<summary> Reports progress and success or failure of gimbal axis calibration procedure |Gimbal axis we're reporting calibration progress for| Current calibration progress for this axis, 0x64=100%| Status of the calibration| Empty| Empty| Empty| Empty|  </summary>
        GIMBAL_AXIS_CALIBRATION_STATUS=42502, 
    	///<summary> Starts commutation calibration on the gimbal |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
        GIMBAL_REQUEST_AXIS_CALIBRATION=42503, 
    	///<summary> Erases gimbal application and parameters |Magic number| Magic number| Magic number| Magic number| Magic number| Magic number| Magic number|  </summary>
        GIMBAL_FULL_RESET=42505, 
    
    };





///<summary>  </summary>
enum MAV_COMPONENT
{
			///<summary>  | </summary>
        MAV_COMP_ID_ALL=0, 
    	///<summary>  | </summary>
        MAV_COMP_ID_CAMERA=100, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO1=140, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO2=141, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO3=142, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO4=143, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO5=144, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO6=145, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO7=146, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO8=147, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO9=148, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO10=149, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO11=150, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO12=151, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO13=152, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SERVO14=153, 
    	///<summary>  | </summary>
        MAV_COMP_ID_GIMBAL=154, 
    	///<summary>  | </summary>
        MAV_COMP_ID_LOG=155, 
    	///<summary>  | </summary>
        MAV_COMP_ID_ADSB=156, 
    	///<summary> On Screen Display (OSD) devices for video links | </summary>
        MAV_COMP_ID_OSD=157, 
    	///<summary> Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter sub-protocol | </summary>
        MAV_COMP_ID_PERIPHERAL=158, 
    	///<summary>  | </summary>
        MAV_COMP_ID_QX1_GIMBAL=159, 
    	///<summary>  | </summary>
        MAV_COMP_ID_MAPPER=180, 
    	///<summary>  | </summary>
        MAV_COMP_ID_MISSIONPLANNER=190, 
    	///<summary>  | </summary>
        MAV_COMP_ID_PATHPLANNER=195, 
    	///<summary>  | </summary>
        MAV_COMP_ID_IMU=200, 
    	///<summary>  | </summary>
        MAV_COMP_ID_IMU_2=201, 
    	///<summary>  | </summary>
        MAV_COMP_ID_IMU_3=202, 
    	///<summary>  | </summary>
        MAV_COMP_ID_GPS=220, 
    	///<summary>  | </summary>
        MAV_COMP_ID_UDP_BRIDGE=240, 
    	///<summary>  | </summary>
        MAV_COMP_ID_UART_BRIDGE=241, 
    	///<summary>  | </summary>
        MAV_COMP_ID_SYSTEM_CONTROL=250, 
    	///<summary>  | </summary>
    
};		


enum MAV_FRAME
{
			///<summary> Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | </summary>
        GLOBAL=0, 
    	///<summary> Local coordinate frame, Z-up (x: north, y: east, z: down). | </summary>
        LOCAL_NED=1, 
    	///<summary> NOT a coordinate frame, indicates a mission command. | </summary>
        MISSION=2, 
    	///<summary> Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | </summary>
        GLOBAL_RELATIVE_ALT=3, 
    	///<summary> Local coordinate frame, Z-down (x: east, y: north, z: up) | </summary>
        LOCAL_ENU=4, 
    	///<summary> Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | </summary>
        GLOBAL_INT=5, 
    	///<summary> Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | </summary>
        GLOBAL_RELATIVE_ALT_INT=6, 
    	///<summary> Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | </summary>
        LOCAL_OFFSET_NED=7, 
    	///<summary> Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | </summary>
        BODY_NED=8, 
    	///<summary> Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | </summary>
        BODY_OFFSET_NED=9, 
    	///<summary> Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | </summary>
        GLOBAL_TERRAIN_ALT=10, 
    	///<summary> Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | </summary>
        GLOBAL_TERRAIN_ALT_INT=11, 
    	///<summary>  | </summary>
    
};
enum MAV_MISSION_RESULT
{
				
			///<summary> mission accepted OK | </summary>
        MAV_MISSION_ACCEPTED=0, 
    	///<summary> generic error / not accepting mission commands at all right now | </summary>
        MAV_MISSION_ERROR=1, 
    	///<summary> coordinate frame is not supported | </summary>
        MAV_MISSION_UNSUPPORTED_FRAME=2, 
    	///<summary> command is not supported | </summary>
        MAV_MISSION_UNSUPPORTED=3, 
    	///<summary> mission item exceeds storage space | </summary>
        MAV_MISSION_NO_SPACE=4, 
    	///<summary> one of the parameters has an invalid value | </summary>
        MAV_MISSION_INVALID=5, 
    	///<summary> param1 has an invalid value | </summary>
        MAV_MISSION_INVALID_PARAM1=6, 
    	///<summary> param2 has an invalid value | </summary>
        MAV_MISSION_INVALID_PARAM2=7, 
    	///<summary> param3 has an invalid value | </summary>
        MAV_MISSION_INVALID_PARAM3=8, 
    	///<summary> param4 has an invalid value | </summary>
        MAV_MISSION_INVALID_PARAM4=9, 
    	///<summary> x/param5 has an invalid value | </summary>
        MAV_MISSION_INVALID_PARAM5_X=10, 
    	///<summary> y/param6 has an invalid value | </summary>
        MAV_MISSION_INVALID_PARAM6_Y=11, 
    	///<summary> param7 has an invalid value | </summary>
        MAV_MISSION_INVALID_PARAM7=12, 
    	///<summary> received waypoint out of sequence | </summary>
        MAV_MISSION_INVALID_SEQUENCE=13, 
    	///<summary> not accepting any mission commands from this communication partner | </summary>
        MAV_MISSION_DENIED=14, 
    	///<summary>  | </summary>
    
 };
//MAVLINK 协议定义相关(字符、长度)
enum MAVLINK_CH_LEN
{

	 MAVLINK_MAX_PAYLOAD_LEN = 255,
	 MAVLINK_CORE_HEADER_LEN = 9,///< Length of core header (of the comm. layer)
	 MAVLINK_CORE_HEADER_MAVLINK1_LEN = 5,//< Length of MAVLink1 core header (of the comm. layer)
	 MAVLINK_NUM_HEADER_BYTES = (MAVLINK_CORE_HEADER_LEN + 1),///< Length of all header bytes, including core and stx
	 MAVLINK_NUM_CHECKSUM_BYTES = 2,
	 MAVLINK_SIGNATURE_BLOCK_LEN = 13,
	 MAVLINK_NUM_NON_PAYLOAD_BYTES = (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES),
	 MAVLINK_MAX_PACKET_LEN = (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_SIGNATURE_BLOCK_LEN),///< Maximum packet length
	 MAVLINK_LITTLE_ENDIAN = 1,
	 MAVLINK_BIG_ENDIAN = 0,
	 MAVLINK_STX = 253,
	 MAVLINK_STX_MAVLINK1 = 0xFE,
	 MAVLINK_ENDIAN = MAVLINK_LITTLE_ENDIAN,
	 MAVLINK_ALIGNED_FIELDS = (1 == 1),
	 MAVLINK_CRC_EXTRA = 1,
	 MAVLINK_COMMAND_24BIT = 1,
	 MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN)
};
//MAVLINK 协议信息ID
enum MAVLINK_MSG_ID
{
	HEARTBEAT = 0,
	SYS_STATUS = 1,
	SYSTEM_TIME = 2,
	PING = 4,
	CHANGE_OPERATOR_CONTROL = 5,
	CHANGE_OPERATOR_CONTROL_ACK = 6,
	AUTH_KEY = 7,
	SET_MODE = 11,
	PARAM_REQUEST_READ = 20,
	PARAM_REQUEST_LIST = 21,
	PARAM_VALUE = 22,
	PARAM_SET = 23,
	GPS_RAW_INT = 24,
	GPS_STATUS = 25,
	SCALED_IMU = 26,
	RAW_IMU = 27,
	RAW_PRESSURE = 28,
	SCALED_PRESSURE = 29,
	ATTITUDE = 30,
	ATTITUDE_QUATERNION = 31,
	LOCAL_POSITION_NED = 32,
	GLOBAL_POSITION_INT = 33,
	RC_CHANNELS_SCALED = 34,
	RC_CHANNELS_RAW = 35,
	SERVO_OUTPUT_RAW = 36,
	MISSION_REQUEST_PARTIAL_LIST = 37,
	MISSION_WRITE_PARTIAL_LIST = 38,
	MISSION_ITEM = 39,
	MISSION_REQUEST = 40,
	MISSION_SET_CURRENT = 41,
	MISSION_CURRENT = 42,
	MISSION_REQUEST_LIST = 43,
	MISSION_COUNT = 44,
	MISSION_CLEAR_ALL = 45,
	MISSION_ITEM_REACHED = 46,
	MISSION_ACK = 47,
	SET_GPS_GLOBAL_ORIGIN = 48,
	GPS_GLOBAL_ORIGIN = 49,
	PARAM_MAP_RC = 50,
	MISSION_REQUEST_INT = 51,
	SAFETY_SET_ALLOWED_AREA = 54,
	SAFETY_ALLOWED_AREA = 55,
	ATTITUDE_QUATERNION_COV = 61,
	NAV_CONTROLLER_OUTPUT = 62,
	GLOBAL_POSITION_INT_COV = 63,
	LOCAL_POSITION_NED_COV = 64,
	RC_CHANNELS = 65,
	REQUEST_DATA_STREAM = 66,
	DATA_STREAM = 67,
	MANUAL_CONTROL = 69,
	RC_CHANNELS_OVERRIDE = 70,
	MISSION_ITEM_INT = 73,
	VFR_HUD = 74,
	COMMAND_INT = 75,
	COMMAND_LONG = 76,
	COMMAND_ACK = 77,
	MANUAL_SETPOINT = 81,
	SET_ATTITUDE_TARGET = 82,
	ATTITUDE_TARGET = 83,
	SET_POSITION_TARGET_LOCAL_NED = 84,
	POSITION_TARGET_LOCAL_NED = 85,
	SET_POSITION_TARGET_GLOBAL_INT = 86,
	POSITION_TARGET_GLOBAL_INT = 87,
	LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = 89,
	HIL_STATE = 90,
	HIL_CONTROLS = 91,
	HIL_RC_INPUTS_RAW = 92,
	OPTICAL_FLOW = 100,
	GLOBAL_VISION_POSITION_ESTIMATE = 101,
	VISION_POSITION_ESTIMATE = 102,
	VISION_SPEED_ESTIMATE = 103,
	VICON_POSITION_ESTIMATE = 104,
	HIGHRES_IMU = 105,
	OPTICAL_FLOW_RAD = 106,
	HIL_SENSOR = 107,
	SIM_STATE = 108,
	RADIO_STATUS = 109,
	FILE_TRANSFER_PROTOCOL = 110,
	TIMESYNC = 111,
	CAMERA_TRIGGER = 112,
	HIL_GPS = 113,
	HIL_OPTICAL_FLOW = 114,
	HIL_STATE_QUATERNION = 115,
	SCALED_IMU2 = 116,
	LOG_REQUEST_LIST = 117,
	LOG_ENTRY = 118,
	LOG_REQUEST_DATA = 119,
	LOG_DATA = 120,
	LOG_ERASE = 121,
	LOG_REQUEST_END = 122,
	GPS_INJECT_DATA = 123,
	GPS2_RAW = 124,
	POWER_STATUS = 125,
	SERIAL_CONTROL = 126,
	GPS_RTK = 127,
	GPS2_RTK = 128,
	SCALED_IMU3 = 129,
	DATA_TRANSMISSION_HANDSHAKE = 130,
	ENCAPSULATED_DATA = 131,
	DISTANCE_SENSOR = 132,
	TERRAIN_REQUEST = 133,
	TERRAIN_DATA = 134,
	TERRAIN_CHECK = 135,
	TERRAIN_REPORT = 136,
	SCALED_PRESSURE2 = 137,
	ATT_POS_MOCAP = 138,
	SET_ACTUATOR_CONTROL_TARGET = 139,
	ACTUATOR_CONTROL_TARGET = 140,
	ALTITUDE = 141,
	RESOURCE_REQUEST = 142,
	SCALED_PRESSURE3 = 143,
	FOLLOW_TARGET = 144,
	CONTROL_SYSTEM_STATE = 146,
	BATTERY_STATUS = 147,
	AUTOPILOT_VERSION = 148,
	LANDING_TARGET = 149,
	SENSOR_OFFSETS = 150,
	SET_MAG_OFFSETS = 151,
	MEMINFO = 152,
	AP_ADC = 153,
	DIGICAM_CONFIGURE = 154,
	DIGICAM_CONTROL = 155,
	MOUNT_CONFIGURE = 156,
	MOUNT_CONTROL = 157,
	MOUNT_STATUS = 158,
	FENCE_POINT = 160,
	FENCE_FETCH_POINT = 161,
	FENCE_STATUS = 162,
	AHRS = 163,
	SIMSTATE = 164,
	HWSTATUS = 165,
	RADIO = 166,
	LIMITS_STATUS = 167,
	WIND = 168,
	DATA16 = 169,
	DATA32 = 170,
	DATA64 = 171,
	DATA96 = 172,
	RANGEFINDER = 173,
	AIRSPEED_AUTOCAL = 174,
	RALLY_POINT = 175,
	RALLY_FETCH_POINT = 176,
	COMPASSMOT_STATUS = 177,
	AHRS2 = 178,
	CAMERA_STATUS = 179,
	CAMERA_FEEDBACK = 180,
	BATTERY2 = 181,
	AHRS3 = 182,
	AUTOPILOT_VERSION_REQUEST = 183,
	REMOTE_LOG_DATA_BLOCK = 184,
	REMOTE_LOG_BLOCK_STATUS = 185,
	LED_CONTROL = 186,
	MAG_CAL_PROGRESS = 191,
	MAG_CAL_REPORT = 192,
	EKF_STATUS_REPORT = 193,
	PID_TUNING = 194,
	GIMBAL_REPORT = 200,
	GIMBAL_CONTROL = 201,
	GIMBAL_TORQUE_CMD_REPORT = 214,
	GOPRO_HEARTBEAT = 215,
	GOPRO_GET_REQUEST = 216,
	GOPRO_GET_RESPONSE = 217,
	GOPRO_SET_REQUEST = 218,
	GOPRO_SET_RESPONSE = 219,
	RPM = 226,
	ESTIMATOR_STATUS = 230,
	WIND_COV = 231,
	GPS_INPUT = 232,
	GPS_RTCM_DATA = 233,
	VIBRATION = 241,
	HOME_POSITION = 242,
	SET_HOME_POSITION = 243,
	MESSAGE_INTERVAL = 244,
	EXTENDED_SYS_STATE = 245,
	ADSB_VEHICLE = 246,
	COLLISION = 247,
	V2_EXTENSION = 248,
	MEMORY_VECT = 249,
	DEBUG_VECT = 250,
	NAMED_VALUE_FLOAT = 251,
	NAMED_VALUE_INT = 252,
	STATUSTEXT = 253,
	DEBUG = 254,
	SETUP_SIGNING = 256,
	BUTTON_CHANGE = 257,
	PLAY_TUNE = 258,
	UAVIONIX_ADSB_OUT_CFG = 10001,
	UAVIONIX_ADSB_OUT_DYNAMIC = 10002,
	UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT = 10003,
};



/**************************************/
/*mavlink解析后的信息结构体
**************************************/
struct mavlink_attitude_t //索引值30
{
	//弧度制
	
	/// <summary> Timestamp (milliseconds since system boot) </summary>
	UINT_32 time_boot_ms;
	/// <summary> Roll angle (rad, -pi..+pi) </summary>
	FLOAT_32 roll;
	/// <summary> Pitch angle (rad, -pi..+pi) </summary>
	FLOAT_32 pitch;
	/// <summary> Yaw angle (rad, -pi..+pi) </summary>
	FLOAT_32 yaw;
	/// <summary> Roll angular speed (rad/s) </summary>
	FLOAT_32 rollspeed;
	/// <summary> Pitch angular speed (rad/s) </summary>
	FLOAT_32 pitchspeed;
	/// <summary> Yaw angular speed (rad/s) </summary>
	FLOAT_32 yawspeed;

};
struct mavlink_global_position_int_t //索引值33
{
	/// <summary> Timestamp (milliseconds since system boot) </summary>
	UINT_32 time_boot_ms;
	/// <summary> Latitude, expressed as degrees * 1E7 </summary>
	Int32 lat;
	/// <summary> Longitude, expressed as degrees * 1E7 </summary>
	Int32 lon;
	/// <summary> Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well) </summary>
	Int32 alt;
	/// <summary> Altitude above ground in meters, expressed as * 1000 (millimeters) </summary>
	Int32 relative_alt;
	/// <summary> Ground X Speed (Latitude, positive north), expressed as m/s * 100 </summary>
	Int16 vx;
	/// <summary> Ground Y Speed (Longitude, positive east), expressed as m/s * 100 </summary>
	Int16 vy;
	/// <summary> Ground Z Speed (Altitude, positive down), expressed as m/s * 100 </summary>
	Int16 vz;
	/// <summary> Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX </summary>
	UInt16 hdg;

};
struct mavlink_vfr_hud_t //索引值74
{
	/// <summary> Current airspeed in m/s </summary>
	FLOAT_32 airspeed;
	/// <summary> Current ground speed in m/s </summary>
	FLOAT_32 groundspeed;
	/// <summary> Current altitude (MSL), in meters </summary>
	FLOAT_32 alt;
	/// <summary> Current climb rate in meters/second </summary>
	FLOAT_32 climb;
	/// <summary> Current heading in degrees, in compass units (0..360, 0=north) </summary>
	INT_16 heading;
	/// <summary> Current throttle setting in integer percent, 0 to 100 </summary>
	UINT_16 throttle;

};
/*****************end******************/ 

//[StructLayout(LayoutKind.Sequential,Pack=1,Size=18)]
struct mavlink_rc_channels_override_t
{
	/// <summary> RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
	UInt16 chan1_raw;
	/// <summary> RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
	UInt16 chan2_raw;
	/// <summary> RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
	UInt16 chan3_raw;
	/// <summary> RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
	UInt16 chan4_raw;
	/// <summary> RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
	UInt16 chan5_raw;
	/// <summary> RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
	UInt16 chan6_raw;
	/// <summary> RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
	UInt16 chan7_raw;
	/// <summary> RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field. </summary>
   UInt16 chan8_raw;
	/// <summary> System ID </summary>
   byte target_system;
	/// <summary> Component ID </summary>
   byte target_component;
    
};

// [StructLayout(LayoutKind.Sequential,Pack=1,Size=6)]
 struct mavlink_set_mode_t
{
  /// <summary> The new autopilot-specific mode. This field can be ignored by an autopilot. </summary>
     UInt32 custom_mode;
  /// <summary> The system setting the mode </summary>
     byte target_system;
  /// <summary> The new base mode </summary>
     byte base_mode;
    
};

// [StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]
struct mavlink_mission_request_t
{
		/// <summary> Sequence </summary>
	  UInt16 seq;
				/// <summary> System ID </summary>
		byte target_system;
				/// <summary> Component ID </summary>
	  byte target_component;

};

//[StructLayout(LayoutKind.Sequential,Pack=1,Size=3)] //索引值 47
struct mavlink_mission_ack_t
{
		/// <summary> System ID </summary>
  byte target_system;
				/// <summary> Component ID </summary>
  byte target_component;
				/// <summary> See MAV_MISSION_RESULT enum </summary>
  byte type;
    
 };
//[StructLayout(LayoutKind.Sequential,Pack=1,Size=4)]  //索引值44
struct mavlink_mission_count_t
{
  /// <summary> Number of mission items in the sequence </summary>
  UINT_16 count;
  /// <summary> System ID </summary>
  UINT_8 target_system;
  /// <summary> Component ID </summary>
  UINT_8 target_component;
    
};

//    [StructLayout(LayoutKind.Sequential,Pack=1,Size=3)]//77
struct mavlink_command_ack_t 
{
        /// <summary> Command ID, as defined by MAV_CMD enum. </summary>
       UInt16 command;
            /// <summary> See MAV_RESULT enum </summary>
        byte result;
    
};


// [StructLayout(LayoutKind.Sequential,Pack=1,Size=33)]// 索引值76
struct mavlink_command_long_t
{
        /// <summary> Parameter 1, as defined by MAV_CMD enum. </summary>
    Single param1;
            /// <summary> Parameter 2, as defined by MAV_CMD enum. </summary>
    Single param2;
            /// <summary> Parameter 3, as defined by MAV_CMD enum. </summary>
    Single param3;
            /// <summary> Parameter 4, as defined by MAV_CMD enum. </summary>
    Single param4;
            /// <summary> Parameter 5, as defined by MAV_CMD enum. </summary>
    Single param5;
            /// <summary> Parameter 6, as defined by MAV_CMD enum. </summary>
    Single param6;
		/// <summary> Parameter 7, as defined by MAV_CMD enum. </summary>
    Single param7;
            /// <summary> Command ID, as defined by MAV_CMD enum. </summary>
    UInt16 command;
            /// <summary> System which should execute the command </summary>
    byte target_system;
            /// <summary> Component which should execute the command, 0 for all components </summary>
    byte target_component;
            /// <summary> 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) </summary>
    byte confirmation;
    
};
//[StructLayout(LayoutKind.Sequential,Pack=1,Size=37)]  //索引值39
struct mavlink_mission_item_t
{
	/// <summary> PARAM1, see MAV_CMD enum </summary>
	FLOAT_32 param1;
		/// <summary> PARAM2, see MAV_CMD enum </summary>
	FLOAT_32 param2;
		/// <summary> PARAM3, see MAV_CMD enum </summary>
	FLOAT_32 param3;
		/// <summary> PARAM4, see MAV_CMD enum </summary>
	FLOAT_32 param4;
		/// <summary> PARAM5 / local: x position, global: latitude </summary>
	FLOAT_32 x;
		/// <summary> PARAM6 / y position: global: longitude </summary>
	FLOAT_32 y;
		/// <summary> PARAM7 / z position: global: altitude (relative or absolute, depending on frame. </summary>
	FLOAT_32 z;
		/// <summary> Sequence </summary>
	UInt16 seq;
		/// <summary> The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs </summary>
	UInt16 command;
		/// <summary> System ID </summary>
	UINT_8 target_system;
		/// <summary> Component ID </summary>
	UINT_8 target_component;
		/// <summary> The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h </summary>
	UINT_8 frame;
		/// <summary> false:0, true:1 </summary>
	UINT_8 current;
		/// <summary> autocontinue to next wp </summary>
	UINT_8 autocontinue;
    
};


//[StructLayout(LayoutKind.Sequential,Pack=1,Size=37)] //索引值73
struct mavlink_mission_item_int_t
{
 /// <summary> PARAM1, see MAV_CMD enum </summary>
		Single param1;
			/// <summary> PARAM2, see MAV_CMD enum </summary>
		Single param2;
			/// <summary> PARAM3, see MAV_CMD enum </summary>
		Single param3;
			/// <summary> PARAM4, see MAV_CMD enum </summary>
		Single param4;
			/// <summary> PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7 </summary>
		Int32 x;
			/// <summary> PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7 </summary>
		Int32 y;
			/// <summary> PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame. </summary>
		Single z;
			/// <summary> Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4). </summary>
		UInt16 seq;
			/// <summary> The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs </summary>
		UInt16 command;
			/// <summary> System ID </summary>
		byte target_system;
			/// <summary> Component ID </summary>
		byte target_component;
			/// <summary> The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h </summary>
		byte frame;
		/// <summary> false:0, true:1 </summary>
		byte current;
			/// <summary> autocontinue to next wp </summary>
		byte autocontinue;

};


struct MAV_STATE
{
	UINT_8 S_1:1;
	UINT_8 S_2:1;
	UINT_8 S_3:1;
	UINT_8 S_4:1;
	UINT_8 S_5:1;
	UINT_8 S_6:1;
	UINT_8 S_7:1;
	UINT_8 S_8:1;
};

//GPS航点设置标志簇
class marvlink_flag  
{
	public	: 
						marvlink_flag(){							
							Flag_sendToPC = 0;
							Flag_sendToAPM = 0;
						}
						
						MAV_STATE WpCommond;
								
						UINT_8 Flag_sendToPC ;
						UINT_8 Flag_sendToAPM;
							
						int AnalyResult;
						
						
};


//GPS航点坐标暂存
struct Locationwp
 {
	 UINT_16 id;				// command id
   byte options;
   float p1;				// param 1
   float p2;				// param 2
   float p3;				// param 3
   float p4;				// param 4
   double lat;				// Lattitude * 10**7
   double lng;				// Longitude * 10**7
   float alt;				// Altitude in centimeters (meters * 100)
};
 
 

//解析结构体组
typedef struct
{
	mavlink_attitude_t attitude_t;
	//UINT_8 Flag_attitude_t;
	
	mavlink_global_position_int_t global_position_int_t;
	//UINT_8 Flag_global_position_int_t;
	
	mavlink_vfr_hud_t vfr_hud_t;
	//UINT_8 Flag_vfr_hud_t;
	
	mavlink_mission_ack_t mission_ack_t;
	//UINT_8 Flag_mission_ack_t;
	
	mavlink_mission_request_t mission_request_t ;
	
}msgGroup;


//MAVLINK 协议解析类
class marvlink_func
{
	public:
					
					marvlink_func(){

												}
									
					static const int X25_INIT_CRC =  0xffff;
					static const int X25_VALIDATE_CRC = 0xf0b8;
					
				  UINT_16 crc_accumulate(UINT_8 b, UINT_16 crc)
					{

                UINT_8 ch = (UINT_8)(b ^ (UINT_8)(crc & 0x00ff));
                ch = (UINT_8)(ch ^ (ch << 4));
                return (UINT_16)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
					}
				  UINT_16 crc_calculate(UINT_8* pBuffer, int length)
					{
            if (length < 1)
            {
                return 0xffff;
            }
            // For a "message" of length bytes contained in the unsigned char array
            // pointed to by pBuffer, calculate the CRC
            // crcCalculate(unsigned char* pBuffer, int length, unsigned short* checkConst) < not needed
            UINT_16 crcTmp;
            int i;

            crcTmp = X25_INIT_CRC;

            for (i = 1; i < length; i++) // skips header
            {
                crcTmp = crc_accumulate(pBuffer[i], crcTmp);
            
            }

            return (crcTmp);
        }
					template <typename T>
					void generatePacket(int messageType, T indata, int sysid, int compid, bool forcemavlink2 , bool forcesigning );
				
					template <typename T>
					void generatePacket(int messageType, T indata)
					{        
            generatePacket(messageType,indata, MAV_GCS_sysid, MAV_GCS_WP_comid ,false,false);
					}
					
					//解析板命令插入发送函数
					void INSERT_SETWP(UINT_8 *buffer,UINT_32 bufferlen);					 	
					
					void setWPTotal(UINT_16 wp_total);
					
					MAV_MISSION_RESULT setWP(mavlink_mission_item_t req);
					
					MAV_MISSION_RESULT setWP(Locationwp loc, UINT_16 index, MAV_FRAME frame, UINT_8 current,UINT_8 autocontinue , bool use_int);
					
					bool Write_RCoverride(void);
					
					bool doCommand(MAV_CMD actionid, float p1, float p2, float p3, float p4, float p5, float p6, float p7);
					
				//armit  == 1时解锁
				//armit  == 0时加锁
					bool doARM(bool armit)
        {
            return doCommand(COMPONENT_ARM_DISARM, armit ? 1 : 0, 21196, 0, 0, 0, 0, 0);
        }
								
					void setWPACK(void);
				
					void setMode(mavlink_set_mode_t mode);
				
					bool setGuidedModeWP(Locationwp gotohere, bool setguidedmode);
				
					mavlink_attitude_t Analyse_ATTITUDE(uint8_t *buffer);
										
					mavlink_global_position_int_t Analyse_GLOBAL_POSITION_INT(uint8_t *buffer);
					
					mavlink_vfr_hud_t Analyse_VFR_HUD(uint8_t *buffer);
					
					mavlink_mission_ack_t Analyse_mission_ack_t(uint8_t *buffer);
					
					mavlink_mission_request_t Analyse_mission_request_t(uint8_t *buffer);
					
	private:



};

/*****************end******************/
extern msgGroup mavMsg;
extern marvlink_func mavMsgFunc;
extern marvlink_flag mavMsgFlag;
#endif

