; Auto-generated. Do not edit!


(cl:in-package fastnav_msgs-msg)


;//! \htmlinclude ObstacleInfo.msg.html

(cl:defclass <ObstacleInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (has_obstacle
    :reader has_obstacle
    :initarg :has_obstacle
    :type cl:boolean
    :initform cl:nil)
   (min_distance
    :reader min_distance
    :initarg :min_distance
    :type cl:float
    :initform 0.0)
   (front_distance
    :reader front_distance
    :initarg :front_distance
    :type cl:float
    :initform 0.0)
   (left_distance
    :reader left_distance
    :initarg :left_distance
    :type cl:float
    :initform 0.0)
   (right_distance
    :reader right_distance
    :initarg :right_distance
    :type cl:float
    :initform 0.0)
   (rear_distance
    :reader rear_distance
    :initarg :rear_distance
    :type cl:float
    :initform 0.0)
   (nearest_point
    :reader nearest_point
    :initarg :nearest_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (obstacle_point_count
    :reader obstacle_point_count
    :initarg :obstacle_point_count
    :type cl:integer
    :initform 0))
)

(cl:defclass ObstacleInfo (<ObstacleInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fastnav_msgs-msg:<ObstacleInfo> is deprecated: use fastnav_msgs-msg:ObstacleInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstacleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:header-val is deprecated.  Use fastnav_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'has_obstacle-val :lambda-list '(m))
(cl:defmethod has_obstacle-val ((m <ObstacleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:has_obstacle-val is deprecated.  Use fastnav_msgs-msg:has_obstacle instead.")
  (has_obstacle m))

(cl:ensure-generic-function 'min_distance-val :lambda-list '(m))
(cl:defmethod min_distance-val ((m <ObstacleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:min_distance-val is deprecated.  Use fastnav_msgs-msg:min_distance instead.")
  (min_distance m))

(cl:ensure-generic-function 'front_distance-val :lambda-list '(m))
(cl:defmethod front_distance-val ((m <ObstacleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:front_distance-val is deprecated.  Use fastnav_msgs-msg:front_distance instead.")
  (front_distance m))

(cl:ensure-generic-function 'left_distance-val :lambda-list '(m))
(cl:defmethod left_distance-val ((m <ObstacleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:left_distance-val is deprecated.  Use fastnav_msgs-msg:left_distance instead.")
  (left_distance m))

(cl:ensure-generic-function 'right_distance-val :lambda-list '(m))
(cl:defmethod right_distance-val ((m <ObstacleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:right_distance-val is deprecated.  Use fastnav_msgs-msg:right_distance instead.")
  (right_distance m))

(cl:ensure-generic-function 'rear_distance-val :lambda-list '(m))
(cl:defmethod rear_distance-val ((m <ObstacleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:rear_distance-val is deprecated.  Use fastnav_msgs-msg:rear_distance instead.")
  (rear_distance m))

(cl:ensure-generic-function 'nearest_point-val :lambda-list '(m))
(cl:defmethod nearest_point-val ((m <ObstacleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:nearest_point-val is deprecated.  Use fastnav_msgs-msg:nearest_point instead.")
  (nearest_point m))

(cl:ensure-generic-function 'obstacle_point_count-val :lambda-list '(m))
(cl:defmethod obstacle_point_count-val ((m <ObstacleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:obstacle_point_count-val is deprecated.  Use fastnav_msgs-msg:obstacle_point_count instead.")
  (obstacle_point_count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleInfo>) ostream)
  "Serializes a message object of type '<ObstacleInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_obstacle) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'front_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rear_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'nearest_point) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_point_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obstacle_point_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'obstacle_point_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'obstacle_point_count)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleInfo>) istream)
  "Deserializes a message object of type '<ObstacleInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'has_obstacle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_distance) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'nearest_point) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_point_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obstacle_point_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'obstacle_point_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'obstacle_point_count)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleInfo>)))
  "Returns string type for a message object of type '<ObstacleInfo>"
  "fastnav_msgs/ObstacleInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleInfo)))
  "Returns string type for a message object of type 'ObstacleInfo"
  "fastnav_msgs/ObstacleInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleInfo>)))
  "Returns md5sum for a message object of type '<ObstacleInfo>"
  "25281cdc1a810d2ecef8ab29564e1d71")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleInfo)))
  "Returns md5sum for a message object of type 'ObstacleInfo"
  "25281cdc1a810d2ecef8ab29564e1d71")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleInfo>)))
  "Returns full string definition for message of type '<ObstacleInfo>"
  (cl:format cl:nil "# FastNav obstacle information.~%# This message is published by fastnav_perception~%# and consumed by fastnav_planner.~%~%std_msgs/Header header~%~%bool has_obstacle~%~%float64 min_distance~%float64 front_distance~%float64 left_distance~%float64 right_distance~%float64 rear_distance~%~%geometry_msgs/Point nearest_point~%~%uint32 obstacle_point_count~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleInfo)))
  "Returns full string definition for message of type 'ObstacleInfo"
  (cl:format cl:nil "# FastNav obstacle information.~%# This message is published by fastnav_perception~%# and consumed by fastnav_planner.~%~%std_msgs/Header header~%~%bool has_obstacle~%~%float64 min_distance~%float64 front_distance~%float64 left_distance~%float64 right_distance~%float64 rear_distance~%~%geometry_msgs/Point nearest_point~%~%uint32 obstacle_point_count~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     8
     8
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'nearest_point))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleInfo
    (cl:cons ':header (header msg))
    (cl:cons ':has_obstacle (has_obstacle msg))
    (cl:cons ':min_distance (min_distance msg))
    (cl:cons ':front_distance (front_distance msg))
    (cl:cons ':left_distance (left_distance msg))
    (cl:cons ':right_distance (right_distance msg))
    (cl:cons ':rear_distance (rear_distance msg))
    (cl:cons ':nearest_point (nearest_point msg))
    (cl:cons ':obstacle_point_count (obstacle_point_count msg))
))
