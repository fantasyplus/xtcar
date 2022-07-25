; Auto-generated. Do not edit!


(cl:in-package behaviour_state_machine_truck-srv)


;//! \htmlinclude GoalPose-request.msg.html

(cl:defclass <GoalPose-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass GoalPose-request (<GoalPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behaviour_state_machine_truck-srv:<GoalPose-request> is deprecated: use behaviour_state_machine_truck-srv:GoalPose-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GoalPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behaviour_state_machine_truck-srv:header-val is deprecated.  Use behaviour_state_machine_truck-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GoalPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behaviour_state_machine_truck-srv:pose-val is deprecated.  Use behaviour_state_machine_truck-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalPose-request>) ostream)
  "Serializes a message object of type '<GoalPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalPose-request>) istream)
  "Deserializes a message object of type '<GoalPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalPose-request>)))
  "Returns string type for a service object of type '<GoalPose-request>"
  "behaviour_state_machine_truck/GoalPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalPose-request)))
  "Returns string type for a service object of type 'GoalPose-request"
  "behaviour_state_machine_truck/GoalPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalPose-request>)))
  "Returns md5sum for a message object of type '<GoalPose-request>"
  "604702dcfaf8996a479f50cf127e8a0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalPose-request)))
  "Returns md5sum for a message object of type 'GoalPose-request"
  "604702dcfaf8996a479f50cf127e8a0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalPose-request>)))
  "Returns full string definition for message of type '<GoalPose-request>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalPose-request)))
  "Returns full string definition for message of type 'GoalPose-request"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalPose-request
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
))
;//! \htmlinclude GoalPose-response.msg.html

(cl:defclass <GoalPose-response> (roslisp-msg-protocol:ros-message)
  ((is_success
    :reader is_success
    :initarg :is_success
    :type cl:boolean
    :initform cl:nil)
   (traj
    :reader traj
    :initarg :traj
    :type mpc_msgs-msg:Lane
    :initform (cl:make-instance 'mpc_msgs-msg:Lane)))
)

(cl:defclass GoalPose-response (<GoalPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behaviour_state_machine_truck-srv:<GoalPose-response> is deprecated: use behaviour_state_machine_truck-srv:GoalPose-response instead.")))

(cl:ensure-generic-function 'is_success-val :lambda-list '(m))
(cl:defmethod is_success-val ((m <GoalPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behaviour_state_machine_truck-srv:is_success-val is deprecated.  Use behaviour_state_machine_truck-srv:is_success instead.")
  (is_success m))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <GoalPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behaviour_state_machine_truck-srv:traj-val is deprecated.  Use behaviour_state_machine_truck-srv:traj instead.")
  (traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalPose-response>) ostream)
  "Serializes a message object of type '<GoalPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_success) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'traj) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalPose-response>) istream)
  "Deserializes a message object of type '<GoalPose-response>"
    (cl:setf (cl:slot-value msg 'is_success) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'traj) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalPose-response>)))
  "Returns string type for a service object of type '<GoalPose-response>"
  "behaviour_state_machine_truck/GoalPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalPose-response)))
  "Returns string type for a service object of type 'GoalPose-response"
  "behaviour_state_machine_truck/GoalPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalPose-response>)))
  "Returns md5sum for a message object of type '<GoalPose-response>"
  "604702dcfaf8996a479f50cf127e8a0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalPose-response)))
  "Returns md5sum for a message object of type 'GoalPose-response"
  "604702dcfaf8996a479f50cf127e8a0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalPose-response>)))
  "Returns full string definition for message of type '<GoalPose-response>"
  (cl:format cl:nil "bool is_success~%mpc_msgs/Lane traj~%~%================================================================================~%MSG: mpc_msgs/Lane~%Header header~%int32 increment~%int32 lane_id~%Waypoint[] waypoints~%~%uint32 lane_index~%float32 cost~%float32 closest_object_distance~%float32 closest_object_velocity~%bool is_blocked~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: mpc_msgs/Waypoint~%# global id~%int32 gid ~%# local id~%int32 lid~%geometry_msgs/PoseStamped pose~%geometry_msgs/TwistStamped twist~%int32 change_flag~%~%uint32 lane_id~%uint32 left_lane_id~%uint32 right_lane_id~%uint32 stop_line_id~%float32 cost~%float32 time_cost~%~%# Lane Direction~%# FORWARD				= 0~%# FORWARD_LEFT	 		= 1~%# FORWARD_RIGHT			= 2~%# BACKWARD				= 3 ~%# BACKWARD_LEFT			= 4~%# BACKWARD_RIGHT		= 5~%# STANDSTILL	 		= 6~%uint32 direction~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalPose-response)))
  "Returns full string definition for message of type 'GoalPose-response"
  (cl:format cl:nil "bool is_success~%mpc_msgs/Lane traj~%~%================================================================================~%MSG: mpc_msgs/Lane~%Header header~%int32 increment~%int32 lane_id~%Waypoint[] waypoints~%~%uint32 lane_index~%float32 cost~%float32 closest_object_distance~%float32 closest_object_velocity~%bool is_blocked~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: mpc_msgs/Waypoint~%# global id~%int32 gid ~%# local id~%int32 lid~%geometry_msgs/PoseStamped pose~%geometry_msgs/TwistStamped twist~%int32 change_flag~%~%uint32 lane_id~%uint32 left_lane_id~%uint32 right_lane_id~%uint32 stop_line_id~%float32 cost~%float32 time_cost~%~%# Lane Direction~%# FORWARD				= 0~%# FORWARD_LEFT	 		= 1~%# FORWARD_RIGHT			= 2~%# BACKWARD				= 3 ~%# BACKWARD_LEFT			= 4~%# BACKWARD_RIGHT		= 5~%# STANDSTILL	 		= 6~%uint32 direction~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalPose-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'traj))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalPose-response
    (cl:cons ':is_success (is_success msg))
    (cl:cons ':traj (traj msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoalPose)))
  'GoalPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoalPose)))
  'GoalPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalPose)))
  "Returns string type for a service object of type '<GoalPose>"
  "behaviour_state_machine_truck/GoalPose")