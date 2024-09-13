#Message for command to robot
# send every few seconds to all robots
# send at a few Hz to manually driven robot

int32 command_sequence_number

# status_time is how often status data is sent from the robot to the base
# video_time is how often video / pointcloud data is sent to the base
float32  status_time
float32  video_time

# standard RC values (982 to 2018uS with 1500uS in the middle)
# pitch - 1400 is forward, 1600 is backwards
# roll - 1700 is right, 1300 is left
# yaw - 1700 is clockwise, 1300 is ccw
# throttle - 982 is zero throttle, 2018 is full power
# chan5 = stabilize vs alt hold mode on drone
float32 pitch
float32 roll
float32 throttle
float32 yaw
float32 chan5
float32 chan6
float32 chan7
float32 chan8

# set the vehicle pose
# increment pose_changed when changing the pose
# set geofence_index to use the x/y/z for a geofence point
# geofence_index of 0 will be ignored
float32 x
float32 y
float32 z
float32 veh_yaw
int8 pose_changed
int8 geofence_index

uint8 base_number
uint8 robot_number
# 0=althold, 1=land, 2=poshold, 3=manual,4=estop,5=auto,6=disarm,7=arm,8=launch,9=leftwall,10=rightwall
uint8 mode


# video_source selects a camera feed or combined camera feeds
uint8    video_source
# video_zoom_location chooses where on the screen to zoom when
# a zoomable camera zoom is selected - first 4 bits is x, second 4 bits is y
# zoom zero or 1 is 1x, 2 is 2x, 3 is 3x and so on
uint8    video_zoom_location
uint8    video_zoom_level

# 0 = ladar based odom, 1 = wheel encoder based odom
# used for when ladar has lost lock or for stairs where ladar will lose lock
uint8 odom_mode

# deploy breadcrumbs
# each time this is incremented, deploy a breadcrumb
uint8 deploy_breadcrumb
