; Auto-generated. Do not edit!


(cl:in-package mpc_msgs-msg)


;//! \htmlinclude TaskStatus.msg.html

(cl:defclass <TaskStatus> (roslisp-msg-protocol:ros-message)
  ((task_end
    :reader task_end
    :initarg :task_end
    :type cl:fixnum
    :initform 0)
   (task_error
    :reader task_error
    :initarg :task_error
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TaskStatus (<TaskStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TaskStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TaskStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mpc_msgs-msg:<TaskStatus> is deprecated: use mpc_msgs-msg:TaskStatus instead.")))

(cl:ensure-generic-function 'task_end-val :lambda-list '(m))
(cl:defmethod task_end-val ((m <TaskStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_msgs-msg:task_end-val is deprecated.  Use mpc_msgs-msg:task_end instead.")
  (task_end m))

(cl:ensure-generic-function 'task_error-val :lambda-list '(m))
(cl:defmethod task_error-val ((m <TaskStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_msgs-msg:task_error-val is deprecated.  Use mpc_msgs-msg:task_error instead.")
  (task_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TaskStatus>) ostream)
  "Serializes a message object of type '<TaskStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_end)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_error)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TaskStatus>) istream)
  "Deserializes a message object of type '<TaskStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_end)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_error)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TaskStatus>)))
  "Returns string type for a message object of type '<TaskStatus>"
  "mpc_msgs/TaskStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskStatus)))
  "Returns string type for a message object of type 'TaskStatus"
  "mpc_msgs/TaskStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TaskStatus>)))
  "Returns md5sum for a message object of type '<TaskStatus>"
  "713ed9c21811428b0f7a78688cdf9573")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TaskStatus)))
  "Returns md5sum for a message object of type 'TaskStatus"
  "713ed9c21811428b0f7a78688cdf9573")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TaskStatus>)))
  "Returns full string definition for message of type '<TaskStatus>"
  (cl:format cl:nil "uint8 task_end~%uint8 task_error~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TaskStatus)))
  "Returns full string definition for message of type 'TaskStatus"
  (cl:format cl:nil "uint8 task_end~%uint8 task_error~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TaskStatus>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TaskStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'TaskStatus
    (cl:cons ':task_end (task_end msg))
    (cl:cons ':task_error (task_error msg))
))
