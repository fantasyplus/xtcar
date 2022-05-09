;; Auto-generated. Do not edit!


(when (boundp 'mpc_msgs::ControlCommand)
  (if (not (find-package "MPC_MSGS"))
    (make-package "MPC_MSGS"))
  (shadow 'ControlCommand (find-package "MPC_MSGS")))
(unless (find-package "MPC_MSGS::CONTROLCOMMAND")
  (make-package "MPC_MSGS::CONTROLCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ControlCommand.msg.html


(defclass mpc_msgs::ControlCommand
  :super ros::object
  :slots (_linear_velocity _linear_acceleration _steering_angle ))

(defmethod mpc_msgs::ControlCommand
  (:init
   (&key
    ((:linear_velocity __linear_velocity) 0.0)
    ((:linear_acceleration __linear_acceleration) 0.0)
    ((:steering_angle __steering_angle) 0.0)
    )
   (send-super :init)
   (setq _linear_velocity (float __linear_velocity))
   (setq _linear_acceleration (float __linear_acceleration))
   (setq _steering_angle (float __steering_angle))
   self)
  (:linear_velocity
   (&optional __linear_velocity)
   (if __linear_velocity (setq _linear_velocity __linear_velocity)) _linear_velocity)
  (:linear_acceleration
   (&optional __linear_acceleration)
   (if __linear_acceleration (setq _linear_acceleration __linear_acceleration)) _linear_acceleration)
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:serialization-length
   ()
   (+
    ;; float32 _linear_velocity
    4
    ;; float32 _linear_acceleration
    4
    ;; float32 _steering_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _linear_velocity
       (sys::poke _linear_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linear_acceleration
       (sys::poke _linear_acceleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _linear_velocity
     (setq _linear_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linear_acceleration
     (setq _linear_acceleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mpc_msgs::ControlCommand :md5sum-) "f18688373335fc84f514359a96a6319d")
(setf (get mpc_msgs::ControlCommand :datatype-) "mpc_msgs/ControlCommand")
(setf (get mpc_msgs::ControlCommand :definition-)
      "float32 linear_velocity
float32 linear_acceleration #m/s^2
float32 steering_angle

")



(provide :mpc_msgs/ControlCommand "f18688373335fc84f514359a96a6319d")


