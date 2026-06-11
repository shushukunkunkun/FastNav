; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude MincoTrajectory.msg.html

(cl:defclass <MincoTrajectory> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (traj_id
    :reader traj_id
    :initarg :traj_id
    :type cl:integer
    :initform 0)
   (order
    :reader order
    :initarg :order
    :type cl:integer
    :initform 0)
   (start_time
    :reader start_time
    :initarg :start_time
    :type cl:real
    :initform 0)
   (touch_goal
    :reader touch_goal
    :initarg :touch_goal
    :type cl:boolean
    :initform cl:nil)
   (duration
    :reader duration
    :initarg :duration
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (coef_x
    :reader coef_x
    :initarg :coef_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (coef_y
    :reader coef_y
    :initarg :coef_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (coef_z
    :reader coef_z
    :initarg :coef_z
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MincoTrajectory (<MincoTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MincoTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MincoTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<MincoTrajectory> is deprecated: use traj_utils-msg:MincoTrajectory instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MincoTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:header-val is deprecated.  Use traj_utils-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'traj_id-val :lambda-list '(m))
(cl:defmethod traj_id-val ((m <MincoTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:traj_id-val is deprecated.  Use traj_utils-msg:traj_id instead.")
  (traj_id m))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <MincoTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:order-val is deprecated.  Use traj_utils-msg:order instead.")
  (order m))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <MincoTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:start_time-val is deprecated.  Use traj_utils-msg:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'touch_goal-val :lambda-list '(m))
(cl:defmethod touch_goal-val ((m <MincoTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:touch_goal-val is deprecated.  Use traj_utils-msg:touch_goal instead.")
  (touch_goal m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <MincoTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:duration-val is deprecated.  Use traj_utils-msg:duration instead.")
  (duration m))

(cl:ensure-generic-function 'coef_x-val :lambda-list '(m))
(cl:defmethod coef_x-val ((m <MincoTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:coef_x-val is deprecated.  Use traj_utils-msg:coef_x instead.")
  (coef_x m))

(cl:ensure-generic-function 'coef_y-val :lambda-list '(m))
(cl:defmethod coef_y-val ((m <MincoTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:coef_y-val is deprecated.  Use traj_utils-msg:coef_y instead.")
  (coef_y m))

(cl:ensure-generic-function 'coef_z-val :lambda-list '(m))
(cl:defmethod coef_z-val ((m <MincoTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:coef_z-val is deprecated.  Use traj_utils-msg:coef_z instead.")
  (coef_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MincoTrajectory>) ostream)
  "Serializes a message object of type '<MincoTrajectory>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'traj_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'traj_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'traj_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'traj_id)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'order)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'start_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'start_time) (cl:floor (cl:slot-value msg 'start_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'touch_goal) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'duration))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coef_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'coef_x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coef_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'coef_y))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coef_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'coef_z))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MincoTrajectory>) istream)
  "Deserializes a message object of type '<MincoTrajectory>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'traj_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'traj_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'traj_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'traj_id)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'order) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:slot-value msg 'touch_goal) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'duration) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'duration)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coef_x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coef_x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coef_y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coef_y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coef_z) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coef_z)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MincoTrajectory>)))
  "Returns string type for a message object of type '<MincoTrajectory>"
  "traj_utils/MincoTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MincoTrajectory)))
  "Returns string type for a message object of type 'MincoTrajectory"
  "traj_utils/MincoTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MincoTrajectory>)))
  "Returns md5sum for a message object of type '<MincoTrajectory>"
  "fc66221f7b12651626bf9f8e8ac1d1e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MincoTrajectory)))
  "Returns md5sum for a message object of type 'MincoTrajectory"
  "fc66221f7b12651626bf9f8e8ac1d1e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MincoTrajectory>)))
  "Returns full string definition for message of type '<MincoTrajectory>"
  (cl:format cl:nil "# FastNav MINCO trajectory message.~%# Planner publishes this message, traj_utils/minco_traj_server samples it into fastnav_msgs/ControlCommand.~%~%std_msgs/Header header~%~%# Monotonic trajectory id generated by planner.~%uint32 traj_id~%~%# Polynomial order. FastNav currently uses GCOPTER Trajectory<5>, so order should be 5.~%int32 order~%~%# Trajectory start time in the planner frame.~%time start_time~%~%# Whether this local trajectory already reaches the final mission goal.~%# If false, traj_server should keep the committed trajectory alive briefly~%# instead of switching to hover immediately at the segment end.~%bool touch_goal~%~%# Piece durations. For $N$ pieces, duration has length $N$.~%float64[] duration~%~%# Per-axis coefficients of each piece.~%# For order 5, each piece owns 6 coefficients. Coefficients are stored in the same order as GCOPTER Piece<5>::CoefficientMat columns.~%# That is, coeff_x[i * 6 + k] is the x-axis coefficient at column $k$ of piece $i$.~%float64[] coef_x~%float64[] coef_y~%float64[] coef_z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MincoTrajectory)))
  "Returns full string definition for message of type 'MincoTrajectory"
  (cl:format cl:nil "# FastNav MINCO trajectory message.~%# Planner publishes this message, traj_utils/minco_traj_server samples it into fastnav_msgs/ControlCommand.~%~%std_msgs/Header header~%~%# Monotonic trajectory id generated by planner.~%uint32 traj_id~%~%# Polynomial order. FastNav currently uses GCOPTER Trajectory<5>, so order should be 5.~%int32 order~%~%# Trajectory start time in the planner frame.~%time start_time~%~%# Whether this local trajectory already reaches the final mission goal.~%# If false, traj_server should keep the committed trajectory alive briefly~%# instead of switching to hover immediately at the segment end.~%bool touch_goal~%~%# Piece durations. For $N$ pieces, duration has length $N$.~%float64[] duration~%~%# Per-axis coefficients of each piece.~%# For order 5, each piece owns 6 coefficients. Coefficients are stored in the same order as GCOPTER Piece<5>::CoefficientMat columns.~%# That is, coeff_x[i * 6 + k] is the x-axis coefficient at column $k$ of piece $i$.~%float64[] coef_x~%float64[] coef_y~%float64[] coef_z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MincoTrajectory>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     8
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'duration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coef_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coef_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coef_z) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MincoTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'MincoTrajectory
    (cl:cons ':header (header msg))
    (cl:cons ':traj_id (traj_id msg))
    (cl:cons ':order (order msg))
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':touch_goal (touch_goal msg))
    (cl:cons ':duration (duration msg))
    (cl:cons ':coef_x (coef_x msg))
    (cl:cons ':coef_y (coef_y msg))
    (cl:cons ':coef_z (coef_z msg))
))
