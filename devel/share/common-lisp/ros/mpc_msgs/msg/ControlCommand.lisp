; Auto-generated. Do not edit!


(cl:in-package mpc_msgs-msg)


;//! \htmlinclude ControlCommand.msg.html

(cl:defclass <ControlCommand> (roslisp-msg-protocol:ros-message)
  ((linear_velocity
    :reader linear_velocity
    :initarg :linear_velocity
    :type cl:float
    :initform 0.0)
   (linear_acceleration
    :reader linear_acceleration
    :initarg :linear_acceleration
    :type cl:float
    :initform 0.0)
   (steering_angle
    :reader steering_angle
    :initarg :steering_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlCommand (<ControlCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mpc_msgs-msg:<ControlCommand> is deprecated: use mpc_msgs-msg:ControlCommand instead.")))

(cl:ensure-generic-function 'linear_velocity-val :lambda-list '(m))
(cl:defmethod linear_velocity-val ((m <ControlCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_msgs-msg:linear_velocity-val is deprecated.  Use mpc_msgs-msg:linear_velocity instead.")
  (linear_velocity m))

(cl:ensure-generic-function 'linear_acceleration-val :lambda-list '(m))
(cl:defmethod linear_acceleration-val ((m <ControlCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_msgs-msg:linear_acceleration-val is deprecated.  Use mpc_msgs-msg:linear_acceleration instead.")
  (linear_acceleration m))

(cl:ensure-generic-function 'steering_angle-val :lambda-list '(m))
(cl:defmethod steering_angle-val ((m <ControlCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_msgs-msg:steering_angle-val is deprecated.  Use mpc_msgs-msg:steering_angle instead.")
  (steering_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlCommand>) ostream)
  "Serializes a message object of type '<ControlCommand>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear_acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlCommand>) istream)
  "Deserializes a message object of type '<ControlCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_acceleration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlCommand>)))
  "Returns string type for a message object of type '<ControlCommand>"
  "mpc_msgs/ControlCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlCommand)))
  "Returns string type for a message object of type 'ControlCommand"
  "mpc_msgs/ControlCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlCommand>)))
  "Returns md5sum for a message object of type '<ControlCommand>"
  "f18688373335fc84f514359a96a6319d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlCommand)))
  "Returns md5sum for a message object of type 'ControlCommand"
  "f18688373335fc84f514359a96a6319d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlCommand>)))
  "Returns full string definition for message of type '<ControlCommand>"
  (cl:format cl:nil "float32 linear_velocity~%float32 linear_acceleration #m/s^2~%float32 steering_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlCommand)))
  "Returns full string definition for message of type 'ControlCommand"
  (cl:format cl:nil "float32 linear_velocity~%float32 linear_acceleration #m/s^2~%float32 steering_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlCommand>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlCommand
    (cl:cons ':linear_velocity (linear_velocity msg))
    (cl:cons ':linear_acceleration (linear_acceleration msg))
    (cl:cons ':steering_angle (steering_angle msg))
))
