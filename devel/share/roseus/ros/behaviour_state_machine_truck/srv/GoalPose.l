;; Auto-generated. Do not edit!


(when (boundp 'behaviour_state_machine_truck::GoalPose)
  (if (not (find-package "BEHAVIOUR_STATE_MACHINE_TRUCK"))
    (make-package "BEHAVIOUR_STATE_MACHINE_TRUCK"))
  (shadow 'GoalPose (find-package "BEHAVIOUR_STATE_MACHINE_TRUCK")))
(unless (find-package "BEHAVIOUR_STATE_MACHINE_TRUCK::GOALPOSE")
  (make-package "BEHAVIOUR_STATE_MACHINE_TRUCK::GOALPOSE"))
(unless (find-package "BEHAVIOUR_STATE_MACHINE_TRUCK::GOALPOSEREQUEST")
  (make-package "BEHAVIOUR_STATE_MACHINE_TRUCK::GOALPOSEREQUEST"))
(unless (find-package "BEHAVIOUR_STATE_MACHINE_TRUCK::GOALPOSERESPONSE")
  (make-package "BEHAVIOUR_STATE_MACHINE_TRUCK::GOALPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(if (not (find-package "MPC_MSGS"))
  (ros::roseus-add-msgs "mpc_msgs"))


(defclass behaviour_state_machine_truck::GoalPoseRequest
  :super ros::object
  :slots (_header _pose ))

(defmethod behaviour_state_machine_truck::GoalPoseRequest
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass behaviour_state_machine_truck::GoalPoseResponse
  :super ros::object
  :slots (_is_success _traj ))

(defmethod behaviour_state_machine_truck::GoalPoseResponse
  (:init
   (&key
    ((:is_success __is_success) nil)
    ((:traj __traj) (instance mpc_msgs::Lane :init))
    )
   (send-super :init)
   (setq _is_success __is_success)
   (setq _traj __traj)
   self)
  (:is_success
   (&optional __is_success)
   (if __is_success (setq _is_success __is_success)) _is_success)
  (:traj
   (&rest __traj)
   (if (keywordp (car __traj))
       (send* _traj __traj)
     (progn
       (if __traj (setq _traj (car __traj)))
       _traj)))
  (:serialization-length
   ()
   (+
    ;; bool _is_success
    1
    ;; mpc_msgs/Lane _traj
    (send _traj :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_success
       (if _is_success (write-byte -1 s) (write-byte 0 s))
     ;; mpc_msgs/Lane _traj
       (send _traj :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_success
     (setq _is_success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; mpc_msgs/Lane _traj
     (send _traj :deserialize buf ptr-) (incf ptr- (send _traj :serialization-length))
   ;;
   self)
  )

(defclass behaviour_state_machine_truck::GoalPose
  :super ros::object
  :slots ())

(setf (get behaviour_state_machine_truck::GoalPose :md5sum-) "604702dcfaf8996a479f50cf127e8a0d")
(setf (get behaviour_state_machine_truck::GoalPose :datatype-) "behaviour_state_machine_truck/GoalPose")
(setf (get behaviour_state_machine_truck::GoalPose :request) behaviour_state_machine_truck::GoalPoseRequest)
(setf (get behaviour_state_machine_truck::GoalPose :response) behaviour_state_machine_truck::GoalPoseResponse)

(defmethod behaviour_state_machine_truck::GoalPoseRequest
  (:response () (instance behaviour_state_machine_truck::GoalPoseResponse :init)))

(setf (get behaviour_state_machine_truck::GoalPoseRequest :md5sum-) "604702dcfaf8996a479f50cf127e8a0d")
(setf (get behaviour_state_machine_truck::GoalPoseRequest :datatype-) "behaviour_state_machine_truck/GoalPoseRequest")
(setf (get behaviour_state_machine_truck::GoalPoseRequest :definition-)
      "std_msgs/Header header
geometry_msgs/Pose pose

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
---
bool is_success
mpc_msgs/Lane traj

================================================================================
MSG: mpc_msgs/Lane
Header header
int32 increment
int32 lane_id
Waypoint[] waypoints

uint32 lane_index
float32 cost
float32 closest_object_distance
float32 closest_object_velocity
bool is_blocked

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
MSG: mpc_msgs/Waypoint
# global id
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
")

(setf (get behaviour_state_machine_truck::GoalPoseResponse :md5sum-) "604702dcfaf8996a479f50cf127e8a0d")
(setf (get behaviour_state_machine_truck::GoalPoseResponse :datatype-) "behaviour_state_machine_truck/GoalPoseResponse")
(setf (get behaviour_state_machine_truck::GoalPoseResponse :definition-)
      "std_msgs/Header header
geometry_msgs/Pose pose

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
---
bool is_success
mpc_msgs/Lane traj

================================================================================
MSG: mpc_msgs/Lane
Header header
int32 increment
int32 lane_id
Waypoint[] waypoints

uint32 lane_index
float32 cost
float32 closest_object_distance
float32 closest_object_velocity
bool is_blocked

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
MSG: mpc_msgs/Waypoint
# global id
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
")



(provide :behaviour_state_machine_truck/GoalPose "604702dcfaf8996a479f50cf127e8a0d")


