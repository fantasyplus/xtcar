; Auto-generated. Do not edit!


(cl:in-package behaviour_state_machine-srv)


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
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behaviour_state_machine-srv:<GoalPose-request> is deprecated: use behaviour_state_machine-srv:GoalPose-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GoalPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behaviour_state_machine-srv:header-val is deprecated.  Use behaviour_state_machine-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GoalPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behaviour_state_machine-srv:pose-val is deprecated.  Use behaviour_state_machine-srv:pose instead.")
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
  "behaviour_state_machine/GoalPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalPose-request)))
  "Returns string type for a service object of type 'GoalPose-request"
  "behaviour_state_machine/GoalPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalPose-request>)))
  "Returns md5sum for a message object of type '<GoalPose-request>"
  "0aad1829ef886d046d8951a77cf3842c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalPose-request)))
  "Returns md5sum for a message object of type 'GoalPose-request"
  "0aad1829ef886d046d8951a77cf3842c")
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
    :initform cl:nil))
)

(cl:defclass GoalPose-response (<GoalPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behaviour_state_machine-srv:<GoalPose-response> is deprecated: use behaviour_state_machine-srv:GoalPose-response instead.")))

(cl:ensure-generic-function 'is_success-val :lambda-list '(m))
(cl:defmethod is_success-val ((m <GoalPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behaviour_state_machine-srv:is_success-val is deprecated.  Use behaviour_state_machine-srv:is_success instead.")
  (is_success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalPose-response>) ostream)
  "Serializes a message object of type '<GoalPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalPose-response>) istream)
  "Deserializes a message object of type '<GoalPose-response>"
    (cl:setf (cl:slot-value msg 'is_success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalPose-response>)))
  "Returns string type for a service object of type '<GoalPose-response>"
  "behaviour_state_machine/GoalPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalPose-response)))
  "Returns string type for a service object of type 'GoalPose-response"
  "behaviour_state_machine/GoalPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalPose-response>)))
  "Returns md5sum for a message object of type '<GoalPose-response>"
  "0aad1829ef886d046d8951a77cf3842c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalPose-response)))
  "Returns md5sum for a message object of type 'GoalPose-response"
  "0aad1829ef886d046d8951a77cf3842c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalPose-response>)))
  "Returns full string definition for message of type '<GoalPose-response>"
  (cl:format cl:nil "bool is_success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalPose-response)))
  "Returns full string definition for message of type 'GoalPose-response"
  (cl:format cl:nil "bool is_success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalPose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalPose-response
    (cl:cons ':is_success (is_success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoalPose)))
  'GoalPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoalPose)))
  'GoalPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalPose)))
  "Returns string type for a service object of type '<GoalPose>"
  "behaviour_state_machine/GoalPose")