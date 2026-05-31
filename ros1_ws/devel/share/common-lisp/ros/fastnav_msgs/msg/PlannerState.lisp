; Auto-generated. Do not edit!


(cl:in-package fastnav_msgs-msg)


;//! \htmlinclude PlannerState.msg.html

(cl:defclass <PlannerState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (state_name
    :reader state_name
    :initarg :state_name
    :type cl:string
    :initform "")
   (current_goal
    :reader current_goal
    :initarg :current_goal
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (goal_reached
    :reader goal_reached
    :initarg :goal_reached
    :type cl:boolean
    :initform cl:nil)
   (has_obstacle
    :reader has_obstacle
    :initarg :has_obstacle
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PlannerState (<PlannerState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlannerState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlannerState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fastnav_msgs-msg:<PlannerState> is deprecated: use fastnav_msgs-msg:PlannerState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PlannerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:header-val is deprecated.  Use fastnav_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <PlannerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:state-val is deprecated.  Use fastnav_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'state_name-val :lambda-list '(m))
(cl:defmethod state_name-val ((m <PlannerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:state_name-val is deprecated.  Use fastnav_msgs-msg:state_name instead.")
  (state_name m))

(cl:ensure-generic-function 'current_goal-val :lambda-list '(m))
(cl:defmethod current_goal-val ((m <PlannerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:current_goal-val is deprecated.  Use fastnav_msgs-msg:current_goal instead.")
  (current_goal m))

(cl:ensure-generic-function 'goal_reached-val :lambda-list '(m))
(cl:defmethod goal_reached-val ((m <PlannerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:goal_reached-val is deprecated.  Use fastnav_msgs-msg:goal_reached instead.")
  (goal_reached m))

(cl:ensure-generic-function 'has_obstacle-val :lambda-list '(m))
(cl:defmethod has_obstacle-val ((m <PlannerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastnav_msgs-msg:has_obstacle-val is deprecated.  Use fastnav_msgs-msg:has_obstacle instead.")
  (has_obstacle m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<PlannerState>)))
    "Constants for message type '<PlannerState>"
  '((:STATE_IDLE . 0)
    (:STATE_TAKEOFF . 1)
    (:STATE_NAVIGATE . 2)
    (:STATE_AVOID . 3)
    (:STATE_HOVER . 4)
    (:STATE_GOAL_REACHED . 5)
    (:STATE_EMERGENCY . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'PlannerState)))
    "Constants for message type 'PlannerState"
  '((:STATE_IDLE . 0)
    (:STATE_TAKEOFF . 1)
    (:STATE_NAVIGATE . 2)
    (:STATE_AVOID . 3)
    (:STATE_HOVER . 4)
    (:STATE_GOAL_REACHED . 5)
    (:STATE_EMERGENCY . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlannerState>) ostream)
  "Serializes a message object of type '<PlannerState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state_name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'current_goal) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'goal_reached) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_obstacle) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlannerState>) istream)
  "Deserializes a message object of type '<PlannerState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'current_goal) istream)
    (cl:setf (cl:slot-value msg 'goal_reached) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'has_obstacle) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlannerState>)))
  "Returns string type for a message object of type '<PlannerState>"
  "fastnav_msgs/PlannerState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlannerState)))
  "Returns string type for a message object of type 'PlannerState"
  "fastnav_msgs/PlannerState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlannerState>)))
  "Returns md5sum for a message object of type '<PlannerState>"
  "2f756c3d9a0a66ce8958743e6cc2f690")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlannerState)))
  "Returns md5sum for a message object of type 'PlannerState"
  "2f756c3d9a0a66ce8958743e6cc2f690")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlannerState>)))
  "Returns full string definition for message of type '<PlannerState>"
  (cl:format cl:nil "# FastNav planner state.~%# This message is published by fastnav_planner~%# for debugging and visualization.~%~%std_msgs/Header header~%~%uint8 STATE_IDLE        = 0~%uint8 STATE_TAKEOFF     = 1~%uint8 STATE_NAVIGATE    = 2~%uint8 STATE_AVOID       = 3~%uint8 STATE_HOVER       = 4~%uint8 STATE_GOAL_REACHED = 5~%uint8 STATE_EMERGENCY   = 6~%~%uint8 state~%string state_name~%~%geometry_msgs/Point current_goal~%~%bool goal_reached~%bool has_obstacle~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlannerState)))
  "Returns full string definition for message of type 'PlannerState"
  (cl:format cl:nil "# FastNav planner state.~%# This message is published by fastnav_planner~%# for debugging and visualization.~%~%std_msgs/Header header~%~%uint8 STATE_IDLE        = 0~%uint8 STATE_TAKEOFF     = 1~%uint8 STATE_NAVIGATE    = 2~%uint8 STATE_AVOID       = 3~%uint8 STATE_HOVER       = 4~%uint8 STATE_GOAL_REACHED = 5~%uint8 STATE_EMERGENCY   = 6~%~%uint8 state~%string state_name~%~%geometry_msgs/Point current_goal~%~%bool goal_reached~%bool has_obstacle~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlannerState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'state_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'current_goal))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlannerState>))
  "Converts a ROS message object to a list"
  (cl:list 'PlannerState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':state_name (state_name msg))
    (cl:cons ':current_goal (current_goal msg))
    (cl:cons ':goal_reached (goal_reached msg))
    (cl:cons ':has_obstacle (has_obstacle msg))
))
