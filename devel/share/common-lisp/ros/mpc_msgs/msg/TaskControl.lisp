; Auto-generated. Do not edit!


(cl:in-package mpc_msgs-msg)


;//! \htmlinclude TaskControl.msg.html

(cl:defclass <TaskControl> (roslisp-msg-protocol:ros-message)
  ((traj_end
    :reader traj_end
    :initarg :traj_end
    :type cl:fixnum
    :initform 0)
   (traj_turn
    :reader traj_turn
    :initarg :traj_turn
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TaskControl (<TaskControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TaskControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TaskControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mpc_msgs-msg:<TaskControl> is deprecated: use mpc_msgs-msg:TaskControl instead.")))

(cl:ensure-generic-function 'traj_end-val :lambda-list '(m))
(cl:defmethod traj_end-val ((m <TaskControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_msgs-msg:traj_end-val is deprecated.  Use mpc_msgs-msg:traj_end instead.")
  (traj_end m))

(cl:ensure-generic-function 'traj_turn-val :lambda-list '(m))
(cl:defmethod traj_turn-val ((m <TaskControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_msgs-msg:traj_turn-val is deprecated.  Use mpc_msgs-msg:traj_turn instead.")
  (traj_turn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TaskControl>) ostream)
  "Serializes a message object of type '<TaskControl>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'traj_end)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'traj_turn)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TaskControl>) istream)
  "Deserializes a message object of type '<TaskControl>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'traj_end)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'traj_turn)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TaskControl>)))
  "Returns string type for a message object of type '<TaskControl>"
  "mpc_msgs/TaskControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskControl)))
  "Returns string type for a message object of type 'TaskControl"
  "mpc_msgs/TaskControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TaskControl>)))
  "Returns md5sum for a message object of type '<TaskControl>"
  "bc711d3525457fc9a0066f50f991e5eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TaskControl)))
  "Returns md5sum for a message object of type 'TaskControl"
  "bc711d3525457fc9a0066f50f991e5eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TaskControl>)))
  "Returns full string definition for message of type '<TaskControl>"
  (cl:format cl:nil "uint8 traj_end~%uint8 traj_turn~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TaskControl)))
  "Returns full string definition for message of type 'TaskControl"
  (cl:format cl:nil "uint8 traj_end~%uint8 traj_turn~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TaskControl>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TaskControl>))
  "Converts a ROS message object to a list"
  (cl:list 'TaskControl
    (cl:cons ':traj_end (traj_end msg))
    (cl:cons ':traj_turn (traj_turn msg))
))
