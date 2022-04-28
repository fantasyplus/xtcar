;; Auto-generated. Do not edit!


(when (boundp 'behaviour_state_machine::GoalPose)
  (if (not (find-package "BEHAVIOUR_STATE_MACHINE"))
    (make-package "BEHAVIOUR_STATE_MACHINE"))
  (shadow 'GoalPose (find-package "BEHAVIOUR_STATE_MACHINE")))
(unless (find-package "BEHAVIOUR_STATE_MACHINE::GOALPOSE")
  (make-package "BEHAVIOUR_STATE_MACHINE::GOALPOSE"))
(unless (find-package "BEHAVIOUR_STATE_MACHINE::GOALPOSEREQUEST")
  (make-package "BEHAVIOUR_STATE_MACHINE::GOALPOSEREQUEST"))
(unless (find-package "BEHAVIOUR_STATE_MACHINE::GOALPOSERESPONSE")
  (make-package "BEHAVIOUR_STATE_MACHINE::GOALPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass behaviour_state_machine::GoalPoseRequest
  :super ros::object
  :slots (_header _pose ))

(defmethod behaviour_state_machine::GoalPoseRequest
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

(defclass behaviour_state_machine::GoalPoseResponse
  :super ros::object
  :slots (_is_success ))

(defmethod behaviour_state_machine::GoalPoseResponse
  (:init
   (&key
    ((:is_success __is_success) nil)
    )
   (send-super :init)
   (setq _is_success __is_success)
   self)
  (:is_success
   (&optional __is_success)
   (if __is_success (setq _is_success __is_success)) _is_success)
  (:serialization-length
   ()
   (+
    ;; bool _is_success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_success
       (if _is_success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_success
     (setq _is_success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass behaviour_state_machine::GoalPose
  :super ros::object
  :slots ())

(setf (get behaviour_state_machine::GoalPose :md5sum-) "0aad1829ef886d046d8951a77cf3842c")
(setf (get behaviour_state_machine::GoalPose :datatype-) "behaviour_state_machine/GoalPose")
(setf (get behaviour_state_machine::GoalPose :request) behaviour_state_machine::GoalPoseRequest)
(setf (get behaviour_state_machine::GoalPose :response) behaviour_state_machine::GoalPoseResponse)

(defmethod behaviour_state_machine::GoalPoseRequest
  (:response () (instance behaviour_state_machine::GoalPoseResponse :init)))

(setf (get behaviour_state_machine::GoalPoseRequest :md5sum-) "0aad1829ef886d046d8951a77cf3842c")
(setf (get behaviour_state_machine::GoalPoseRequest :datatype-) "behaviour_state_machine/GoalPoseRequest")
(setf (get behaviour_state_machine::GoalPoseRequest :definition-)
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
")

(setf (get behaviour_state_machine::GoalPoseResponse :md5sum-) "0aad1829ef886d046d8951a77cf3842c")
(setf (get behaviour_state_machine::GoalPoseResponse :datatype-) "behaviour_state_machine/GoalPoseResponse")
(setf (get behaviour_state_machine::GoalPoseResponse :definition-)
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
")



(provide :behaviour_state_machine/GoalPose "0aad1829ef886d046d8951a77cf3842c")


