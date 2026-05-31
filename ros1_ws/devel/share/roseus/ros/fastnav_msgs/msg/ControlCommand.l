;; Auto-generated. Do not edit!


(when (boundp 'fastnav_msgs::ControlCommand)
  (if (not (find-package "FASTNAV_MSGS"))
    (make-package "FASTNAV_MSGS"))
  (shadow 'ControlCommand (find-package "FASTNAV_MSGS")))
(unless (find-package "FASTNAV_MSGS::CONTROLCOMMAND")
  (make-package "FASTNAV_MSGS::CONTROLCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ControlCommand.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*COMMAND_POSITION*" (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(shadow '*COMMAND_POSITION* (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(defconstant fastnav_msgs::ControlCommand::*COMMAND_POSITION* 0)
(intern "*COMMAND_VELOCITY*" (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(shadow '*COMMAND_VELOCITY* (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(defconstant fastnav_msgs::ControlCommand::*COMMAND_VELOCITY* 1)
(intern "*COMMAND_HOVER*" (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(shadow '*COMMAND_HOVER* (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(defconstant fastnav_msgs::ControlCommand::*COMMAND_HOVER* 2)
(intern "*COMMAND_LAND*" (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(shadow '*COMMAND_LAND* (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(defconstant fastnav_msgs::ControlCommand::*COMMAND_LAND* 3)
(intern "*COMMAND_IDLE*" (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(shadow '*COMMAND_IDLE* (find-package "FASTNAV_MSGS::CONTROLCOMMAND"))
(defconstant fastnav_msgs::ControlCommand::*COMMAND_IDLE* 4)

(defun fastnav_msgs::ControlCommand-to-symbol (const)
  (cond
        ((= const 0) 'fastnav_msgs::ControlCommand::*COMMAND_POSITION*)
        ((= const 1) 'fastnav_msgs::ControlCommand::*COMMAND_VELOCITY*)
        ((= const 2) 'fastnav_msgs::ControlCommand::*COMMAND_HOVER*)
        ((= const 3) 'fastnav_msgs::ControlCommand::*COMMAND_LAND*)
        ((= const 4) 'fastnav_msgs::ControlCommand::*COMMAND_IDLE*)
        (t nil)))

(defclass fastnav_msgs::ControlCommand
  :super ros::object
  :slots (_header _command_type _position _velocity _yaw _yaw_rate _enable ))

(defmethod fastnav_msgs::ControlCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:command_type __command_type) 0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:yaw __yaw) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _command_type (round __command_type))
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _yaw (float __yaw))
   (setq _yaw_rate (float __yaw_rate))
   (setq _enable __enable)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:command_type
   (&optional __command_type)
   (if __command_type (setq _command_type __command_type)) _command_type)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:enable
   (&optional (__enable :null))
   (if (not (eq __enable :null)) (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _command_type
    1
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; float64 _yaw
    8
    ;; float64 _yaw_rate
    8
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _command_type
       (write-byte _command_type s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _command_type
     (setq _command_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fastnav_msgs::ControlCommand :md5sum-) "db5bf442422b499bcadde2f7ee9e0236")
(setf (get fastnav_msgs::ControlCommand :datatype-) "fastnav_msgs/ControlCommand")
(setf (get fastnav_msgs::ControlCommand :definition-)
      "# FastNav internal control command.
# This message is published by fastnav_planner
# and consumed by fastnav_control.

std_msgs/Header header

# Command type
uint8 COMMAND_POSITION = 0
uint8 COMMAND_VELOCITY = 1
uint8 COMMAND_HOVER    = 2
uint8 COMMAND_LAND     = 3
uint8 COMMAND_IDLE     = 4

uint8 command_type

# Position setpoint in local frame
geometry_msgs/Point position

# Velocity setpoint in local frame
geometry_msgs/Vector3 velocity

# Yaw angle for position control
float64 yaw

# Yaw rate for velocity control
float64 yaw_rate

# Whether this command is valid
bool enable
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :fastnav_msgs/ControlCommand "db5bf442422b499bcadde2f7ee9e0236")


