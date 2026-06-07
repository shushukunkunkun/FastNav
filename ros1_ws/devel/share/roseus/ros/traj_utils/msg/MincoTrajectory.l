;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::MincoTrajectory)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'MincoTrajectory (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::MINCOTRAJECTORY")
  (make-package "TRAJ_UTILS::MINCOTRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude MincoTrajectory.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass traj_utils::MincoTrajectory
  :super ros::object
  :slots (_header _traj_id _order _start_time _duration _coef_x _coef_y _coef_z ))

(defmethod traj_utils::MincoTrajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:traj_id __traj_id) 0)
    ((:order __order) 0)
    ((:start_time __start_time) (instance ros::time :init))
    ((:duration __duration) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:coef_x __coef_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:coef_y __coef_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:coef_z __coef_z) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _traj_id (round __traj_id))
   (setq _order (round __order))
   (setq _start_time __start_time)
   (setq _duration __duration)
   (setq _coef_x __coef_x)
   (setq _coef_y __coef_y)
   (setq _coef_z __coef_z)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:traj_id
   (&optional __traj_id)
   (if __traj_id (setq _traj_id __traj_id)) _traj_id)
  (:order
   (&optional __order)
   (if __order (setq _order __order)) _order)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:coef_x
   (&optional __coef_x)
   (if __coef_x (setq _coef_x __coef_x)) _coef_x)
  (:coef_y
   (&optional __coef_y)
   (if __coef_y (setq _coef_y __coef_y)) _coef_y)
  (:coef_z
   (&optional __coef_z)
   (if __coef_z (setq _coef_z __coef_z)) _coef_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _traj_id
    4
    ;; int32 _order
    4
    ;; time _start_time
    8
    ;; float64[] _duration
    (* 8    (length _duration)) 4
    ;; float64[] _coef_x
    (* 8    (length _coef_x)) 4
    ;; float64[] _coef_y
    (* 8    (length _coef_y)) 4
    ;; float64[] _coef_z
    (* 8    (length _coef_z)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _traj_id
       (write-long _traj_id s)
     ;; int32 _order
       (write-long _order s)
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; float64[] _duration
     (write-long (length _duration) s)
     (dotimes (i (length _duration))
       (sys::poke (elt _duration i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _coef_x
     (write-long (length _coef_x) s)
     (dotimes (i (length _coef_x))
       (sys::poke (elt _coef_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _coef_y
     (write-long (length _coef_y) s)
     (dotimes (i (length _coef_y))
       (sys::poke (elt _coef_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _coef_z
     (write-long (length _coef_z) s)
     (dotimes (i (length _coef_z))
       (sys::poke (elt _coef_z i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _traj_id
     (setq _traj_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _order
     (setq _order (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _duration
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _duration (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _duration i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _coef_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coef_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coef_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _coef_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coef_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coef_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _coef_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coef_z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coef_z i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get traj_utils::MincoTrajectory :md5sum-) "de89d3d619bcfb3593e56bbf412d2768")
(setf (get traj_utils::MincoTrajectory :datatype-) "traj_utils/MincoTrajectory")
(setf (get traj_utils::MincoTrajectory :definition-)
      "# FastNav MINCO trajectory message.
# Planner publishes this message, traj_utils/minco_traj_server samples it into fastnav_msgs/ControlCommand.

std_msgs/Header header

# Monotonic trajectory id generated by planner.
uint32 traj_id

# Polynomial order. FastNav currently uses GCOPTER Trajectory<5>, so order should be 5.
int32 order

# Trajectory start time in the planner frame.
time start_time

# Piece durations. For $N$ pieces, duration has length $N$.
float64[] duration

# Per-axis coefficients of each piece.
# For order 5, each piece owns 6 coefficients. Coefficients are stored in the same order as GCOPTER Piece<5>::CoefficientMat columns.
# That is, coeff_x[i * 6 + k] is the x-axis coefficient at column $k$ of piece $i$.
float64[] coef_x
float64[] coef_y
float64[] coef_z

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

")



(provide :traj_utils/MincoTrajectory "de89d3d619bcfb3593e56bbf412d2768")


