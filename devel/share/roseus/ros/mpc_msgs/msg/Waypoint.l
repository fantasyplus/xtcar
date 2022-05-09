;; Auto-generated. Do not edit!


(when (boundp 'mpc_msgs::Waypoint)
  (if (not (find-package "MPC_MSGS"))
    (make-package "MPC_MSGS"))
  (shadow 'Waypoint (find-package "MPC_MSGS")))
(unless (find-package "MPC_MSGS::WAYPOINT")
  (make-package "MPC_MSGS::WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Waypoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass mpc_msgs::Waypoint
  :super ros::object
  :slots (_gid _lid _pose _twist _change_flag _lane_id _left_lane_id _right_lane_id _stop_line_id _cost _time_cost _direction ))

(defmethod mpc_msgs::Waypoint
  (:init
   (&key
    ((:gid __gid) 0)
    ((:lid __lid) 0)
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    ((:twist __twist) (instance geometry_msgs::TwistStamped :init))
    ((:change_flag __change_flag) 0)
    ((:lane_id __lane_id) 0)
    ((:left_lane_id __left_lane_id) 0)
    ((:right_lane_id __right_lane_id) 0)
    ((:stop_line_id __stop_line_id) 0)
    ((:cost __cost) 0.0)
    ((:time_cost __time_cost) 0.0)
    ((:direction __direction) 0)
    )
   (send-super :init)
   (setq _gid (round __gid))
   (setq _lid (round __lid))
   (setq _pose __pose)
   (setq _twist __twist)
   (setq _change_flag (round __change_flag))
   (setq _lane_id (round __lane_id))
   (setq _left_lane_id (round __left_lane_id))
   (setq _right_lane_id (round __right_lane_id))
   (setq _stop_line_id (round __stop_line_id))
   (setq _cost (float __cost))
   (setq _time_cost (float __time_cost))
   (setq _direction (round __direction))
   self)
  (:gid
   (&optional __gid)
   (if __gid (setq _gid __gid)) _gid)
  (:lid
   (&optional __lid)
   (if __lid (setq _lid __lid)) _lid)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:change_flag
   (&optional __change_flag)
   (if __change_flag (setq _change_flag __change_flag)) _change_flag)
  (:lane_id
   (&optional __lane_id)
   (if __lane_id (setq _lane_id __lane_id)) _lane_id)
  (:left_lane_id
   (&optional __left_lane_id)
   (if __left_lane_id (setq _left_lane_id __left_lane_id)) _left_lane_id)
  (:right_lane_id
   (&optional __right_lane_id)
   (if __right_lane_id (setq _right_lane_id __right_lane_id)) _right_lane_id)
  (:stop_line_id
   (&optional __stop_line_id)
   (if __stop_line_id (setq _stop_line_id __stop_line_id)) _stop_line_id)
  (:cost
   (&optional __cost)
   (if __cost (setq _cost __cost)) _cost)
  (:time_cost
   (&optional __time_cost)
   (if __time_cost (setq _time_cost __time_cost)) _time_cost)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:serialization-length
   ()
   (+
    ;; int32 _gid
    4
    ;; int32 _lid
    4
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/TwistStamped _twist
    (send _twist :serialization-length)
    ;; int32 _change_flag
    4
    ;; uint32 _lane_id
    4
    ;; uint32 _left_lane_id
    4
    ;; uint32 _right_lane_id
    4
    ;; uint32 _stop_line_id
    4
    ;; float32 _cost
    4
    ;; float32 _time_cost
    4
    ;; uint32 _direction
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _gid
       (write-long _gid s)
     ;; int32 _lid
       (write-long _lid s)
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;; geometry_msgs/TwistStamped _twist
       (send _twist :serialize s)
     ;; int32 _change_flag
       (write-long _change_flag s)
     ;; uint32 _lane_id
       (write-long _lane_id s)
     ;; uint32 _left_lane_id
       (write-long _left_lane_id s)
     ;; uint32 _right_lane_id
       (write-long _right_lane_id s)
     ;; uint32 _stop_line_id
       (write-long _stop_line_id s)
     ;; float32 _cost
       (sys::poke _cost (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _time_cost
       (sys::poke _time_cost (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _direction
       (write-long _direction s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _gid
     (setq _gid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _lid
     (setq _lid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/TwistStamped _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; int32 _change_flag
     (setq _change_flag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _lane_id
     (setq _lane_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _left_lane_id
     (setq _left_lane_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _right_lane_id
     (setq _right_lane_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _stop_line_id
     (setq _stop_line_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _cost
     (setq _cost (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _time_cost
     (setq _time_cost (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _direction
     (setq _direction (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mpc_msgs::Waypoint :md5sum-) "7548551657a1997f59a03c87ccbc5644")
(setf (get mpc_msgs::Waypoint :datatype-) "mpc_msgs/Waypoint")
(setf (get mpc_msgs::Waypoint :definition-)
      "# global id
int32 gid 
# local id
int32 lid
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped twist
int32 change_flag

uint32 lane_id
uint32 left_lane_id
uint32 right_lane_id
uint32 stop_line_id
float32 cost
float32 time_cost

# Lane Direction
# FORWARD				= 0
# FORWARD_LEFT	 		= 1
# FORWARD_RIGHT			= 2
# BACKWARD				= 3 
# BACKWARD_LEFT			= 4
# BACKWARD_RIGHT		= 5
# STANDSTILL	 		= 6
uint32 direction

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistStamped
# A twist with reference coordinate frame and timestamp
Header header
Twist twist

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

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



(provide :mpc_msgs/Waypoint "7548551657a1997f59a03c87ccbc5644")


