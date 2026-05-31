;; Auto-generated. Do not edit!


(when (boundp 'fastnav_msgs::ObstacleInfo)
  (if (not (find-package "FASTNAV_MSGS"))
    (make-package "FASTNAV_MSGS"))
  (shadow 'ObstacleInfo (find-package "FASTNAV_MSGS")))
(unless (find-package "FASTNAV_MSGS::OBSTACLEINFO")
  (make-package "FASTNAV_MSGS::OBSTACLEINFO"))

(in-package "ROS")
;;//! \htmlinclude ObstacleInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fastnav_msgs::ObstacleInfo
  :super ros::object
  :slots (_header _has_obstacle _min_distance _front_distance _left_distance _right_distance _rear_distance _nearest_point _obstacle_point_count ))

(defmethod fastnav_msgs::ObstacleInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:has_obstacle __has_obstacle) nil)
    ((:min_distance __min_distance) 0.0)
    ((:front_distance __front_distance) 0.0)
    ((:left_distance __left_distance) 0.0)
    ((:right_distance __right_distance) 0.0)
    ((:rear_distance __rear_distance) 0.0)
    ((:nearest_point __nearest_point) (instance geometry_msgs::Point :init))
    ((:obstacle_point_count __obstacle_point_count) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _has_obstacle __has_obstacle)
   (setq _min_distance (float __min_distance))
   (setq _front_distance (float __front_distance))
   (setq _left_distance (float __left_distance))
   (setq _right_distance (float __right_distance))
   (setq _rear_distance (float __rear_distance))
   (setq _nearest_point __nearest_point)
   (setq _obstacle_point_count (round __obstacle_point_count))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:has_obstacle
   (&optional (__has_obstacle :null))
   (if (not (eq __has_obstacle :null)) (setq _has_obstacle __has_obstacle)) _has_obstacle)
  (:min_distance
   (&optional __min_distance)
   (if __min_distance (setq _min_distance __min_distance)) _min_distance)
  (:front_distance
   (&optional __front_distance)
   (if __front_distance (setq _front_distance __front_distance)) _front_distance)
  (:left_distance
   (&optional __left_distance)
   (if __left_distance (setq _left_distance __left_distance)) _left_distance)
  (:right_distance
   (&optional __right_distance)
   (if __right_distance (setq _right_distance __right_distance)) _right_distance)
  (:rear_distance
   (&optional __rear_distance)
   (if __rear_distance (setq _rear_distance __rear_distance)) _rear_distance)
  (:nearest_point
   (&rest __nearest_point)
   (if (keywordp (car __nearest_point))
       (send* _nearest_point __nearest_point)
     (progn
       (if __nearest_point (setq _nearest_point (car __nearest_point)))
       _nearest_point)))
  (:obstacle_point_count
   (&optional __obstacle_point_count)
   (if __obstacle_point_count (setq _obstacle_point_count __obstacle_point_count)) _obstacle_point_count)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _has_obstacle
    1
    ;; float64 _min_distance
    8
    ;; float64 _front_distance
    8
    ;; float64 _left_distance
    8
    ;; float64 _right_distance
    8
    ;; float64 _rear_distance
    8
    ;; geometry_msgs/Point _nearest_point
    (send _nearest_point :serialization-length)
    ;; uint32 _obstacle_point_count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _has_obstacle
       (if _has_obstacle (write-byte -1 s) (write-byte 0 s))
     ;; float64 _min_distance
       (sys::poke _min_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _front_distance
       (sys::poke _front_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_distance
       (sys::poke _left_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_distance
       (sys::poke _right_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rear_distance
       (sys::poke _rear_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point _nearest_point
       (send _nearest_point :serialize s)
     ;; uint32 _obstacle_point_count
       (write-long _obstacle_point_count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _has_obstacle
     (setq _has_obstacle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _min_distance
     (setq _min_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _front_distance
     (setq _front_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_distance
     (setq _left_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_distance
     (setq _right_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rear_distance
     (setq _rear_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point _nearest_point
     (send _nearest_point :deserialize buf ptr-) (incf ptr- (send _nearest_point :serialization-length))
   ;; uint32 _obstacle_point_count
     (setq _obstacle_point_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get fastnav_msgs::ObstacleInfo :md5sum-) "25281cdc1a810d2ecef8ab29564e1d71")
(setf (get fastnav_msgs::ObstacleInfo :datatype-) "fastnav_msgs/ObstacleInfo")
(setf (get fastnav_msgs::ObstacleInfo :definition-)
      "# FastNav obstacle information.
# This message is published by fastnav_perception
# and consumed by fastnav_planner.

std_msgs/Header header

bool has_obstacle

float64 min_distance
float64 front_distance
float64 left_distance
float64 right_distance
float64 rear_distance

geometry_msgs/Point nearest_point

uint32 obstacle_point_count
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



(provide :fastnav_msgs/ObstacleInfo "25281cdc1a810d2ecef8ab29564e1d71")


