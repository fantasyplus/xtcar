;; Auto-generated. Do not edit!


(when (boundp 'mpc_msgs::TaskControl)
  (if (not (find-package "MPC_MSGS"))
    (make-package "MPC_MSGS"))
  (shadow 'TaskControl (find-package "MPC_MSGS")))
(unless (find-package "MPC_MSGS::TASKCONTROL")
  (make-package "MPC_MSGS::TASKCONTROL"))

(in-package "ROS")
;;//! \htmlinclude TaskControl.msg.html


(defclass mpc_msgs::TaskControl
  :super ros::object
  :slots (_traj_end _traj_turn ))

(defmethod mpc_msgs::TaskControl
  (:init
   (&key
    ((:traj_end __traj_end) 0)
    ((:traj_turn __traj_turn) 0)
    )
   (send-super :init)
   (setq _traj_end (round __traj_end))
   (setq _traj_turn (round __traj_turn))
   self)
  (:traj_end
   (&optional __traj_end)
   (if __traj_end (setq _traj_end __traj_end)) _traj_end)
  (:traj_turn
   (&optional __traj_turn)
   (if __traj_turn (setq _traj_turn __traj_turn)) _traj_turn)
  (:serialization-length
   ()
   (+
    ;; uint8 _traj_end
    1
    ;; uint8 _traj_turn
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _traj_end
       (write-byte _traj_end s)
     ;; uint8 _traj_turn
       (write-byte _traj_turn s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _traj_end
     (setq _traj_end (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _traj_turn
     (setq _traj_turn (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mpc_msgs::TaskControl :md5sum-) "bc711d3525457fc9a0066f50f991e5eb")
(setf (get mpc_msgs::TaskControl :datatype-) "mpc_msgs/TaskControl")
(setf (get mpc_msgs::TaskControl :definition-)
      "uint8 traj_end
uint8 traj_turn
")



(provide :mpc_msgs/TaskControl "bc711d3525457fc9a0066f50f991e5eb")


