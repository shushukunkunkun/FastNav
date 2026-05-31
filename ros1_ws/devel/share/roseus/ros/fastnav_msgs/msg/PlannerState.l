;; Auto-generated. Do not edit!


(when (boundp 'fastnav_msgs::PlannerState)
  (if (not (find-package "FASTNAV_MSGS"))
    (make-package "FASTNAV_MSGS"))
  (shadow 'PlannerState (find-package "FASTNAV_MSGS")))
(unless (find-package "FASTNAV_MSGS::PLANNERSTATE")
  (make-package "FASTNAV_MSGS::PLANNERSTATE"))

(in-package "ROS")
;;//! \htmlinclude PlannerState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATE_IDLE*" (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(shadow '*STATE_IDLE* (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(defconstant fastnav_msgs::PlannerState::*STATE_IDLE* 0)
(intern "*STATE_TAKEOFF*" (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(shadow '*STATE_TAKEOFF* (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(defconstant fastnav_msgs::PlannerState::*STATE_TAKEOFF* 1)
(intern "*STATE_NAVIGATE*" (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(shadow '*STATE_NAVIGATE* (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(defconstant fastnav_msgs::PlannerState::*STATE_NAVIGATE* 2)
(intern "*STATE_AVOID*" (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(shadow '*STATE_AVOID* (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(defconstant fastnav_msgs::PlannerState::*STATE_AVOID* 3)
(intern "*STATE_HOVER*" (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(shadow '*STATE_HOVER* (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(defconstant fastnav_msgs::PlannerState::*STATE_HOVER* 4)
(intern "*STATE_GOAL_REACHED*" (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(shadow '*STATE_GOAL_REACHED* (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(defconstant fastnav_msgs::PlannerState::*STATE_GOAL_REACHED* 5)
(intern "*STATE_EMERGENCY*" (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(shadow '*STATE_EMERGENCY* (find-package "FASTNAV_MSGS::PLANNERSTATE"))
(defconstant fastnav_msgs::PlannerState::*STATE_EMERGENCY* 6)

(defun fastnav_msgs::PlannerState-to-symbol (const)
  (cond
        ((= const 0) 'fastnav_msgs::PlannerState::*STATE_IDLE*)
        ((= const 1) 'fastnav_msgs::PlannerState::*STATE_TAKEOFF*)
        ((= const 2) 'fastnav_msgs::PlannerState::*STATE_NAVIGATE*)
        ((= const 3) 'fastnav_msgs::PlannerState::*STATE_AVOID*)
        ((= const 4) 'fastnav_msgs::PlannerState::*STATE_HOVER*)
        ((= const 5) 'fastnav_msgs::PlannerState::*STATE_GOAL_REACHED*)
        ((= const 6) 'fastnav_msgs::PlannerState::*STATE_EMERGENCY*)
        (t nil)))

(defclass fastnav_msgs::PlannerState
  :super ros::object
  :slots (_header _state _state_name _current_goal _goal_reached _has_obstacle ))

(defmethod fastnav_msgs::PlannerState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:state_name __state_name) "")
    ((:current_goal __current_goal) (instance geometry_msgs::Point :init))
    ((:goal_reached __goal_reached) nil)
    ((:has_obstacle __has_obstacle) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _state_name (string __state_name))
   (setq _current_goal __current_goal)
   (setq _goal_reached __goal_reached)
   (setq _has_obstacle __has_obstacle)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:state_name
   (&optional __state_name)
   (if __state_name (setq _state_name __state_name)) _state_name)
  (:current_goal
   (&rest __current_goal)
   (if (keywordp (car __current_goal))
       (send* _current_goal __current_goal)
     (progn
       (if __current_goal (setq _current_goal (car __current_goal)))
       _current_goal)))
  (:goal_reached
   (&optional (__goal_reached :null))
   (if (not (eq __goal_reached :null)) (setq _goal_reached __goal_reached)) _goal_reached)
  (:has_obstacle
   (&optional (__has_obstacle :null))
   (if (not (eq __has_obstacle :null)) (setq _has_obstacle __has_obstacle)) _has_obstacle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _state
    1
    ;; string _state_name
    4 (length _state_name)
    ;; geometry_msgs/Point _current_goal
    (send _current_goal :serialization-length)
    ;; bool _goal_reached
    1
    ;; bool _has_obstacle
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _state
       (write-byte _state s)
     ;; string _state_name
       (write-long (length _state_name) s) (princ _state_name s)
     ;; geometry_msgs/Point _current_goal
       (send _current_goal :serialize s)
     ;; bool _goal_reached
       (if _goal_reached (write-byte -1 s) (write-byte 0 s))
     ;; bool _has_obstacle
       (if _has_obstacle (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _state_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _current_goal
     (send _current_goal :deserialize buf ptr-) (incf ptr- (send _current_goal :serialization-length))
   ;; bool _goal_reached
     (setq _goal_reached (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _has_obstacle
     (setq _has_obstacle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fastnav_msgs::PlannerState :md5sum-) "2f756c3d9a0a66ce8958743e6cc2f690")
(setf (get fastnav_msgs::PlannerState :datatype-) "fastnav_msgs/PlannerState")
(setf (get fastnav_msgs::PlannerState :definition-)
      "# FastNav planner state.
# This message is published by fastnav_planner
# for debugging and visualization.

std_msgs/Header header

uint8 STATE_IDLE        = 0
uint8 STATE_TAKEOFF     = 1
uint8 STATE_NAVIGATE    = 2
uint8 STATE_AVOID       = 3
uint8 STATE_HOVER       = 4
uint8 STATE_GOAL_REACHED = 5
uint8 STATE_EMERGENCY   = 6

uint8 state
string state_name

geometry_msgs/Point current_goal

bool goal_reached
bool has_obstacle
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

")



(provide :fastnav_msgs/PlannerState "2f756c3d9a0a66ce8958743e6cc2f690")


