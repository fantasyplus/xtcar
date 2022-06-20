;; Auto-generated. Do not edit!


(when (boundp 'mpc_msgs::TaskStatus)
  (if (not (find-package "MPC_MSGS"))
    (make-package "MPC_MSGS"))
  (shadow 'TaskStatus (find-package "MPC_MSGS")))
(unless (find-package "MPC_MSGS::TASKSTATUS")
  (make-package "MPC_MSGS::TASKSTATUS"))

(in-package "ROS")
;;//! \htmlinclude TaskStatus.msg.html


(defclass mpc_msgs::TaskStatus
  :super ros::object
  :slots (_task_end ))

(defmethod mpc_msgs::TaskStatus
  (:init
   (&key
    ((:task_end __task_end) 0)
    )
   (send-super :init)
   (setq _task_end (round __task_end))
   self)
  (:task_end
   (&optional __task_end)
   (if __task_end (setq _task_end __task_end)) _task_end)
  (:serialization-length
   ()
   (+
    ;; uint8 _task_end
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _task_end
       (write-byte _task_end s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _task_end
     (setq _task_end (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mpc_msgs::TaskStatus :md5sum-) "b511025659d92736cbd31617b8452ad5")
(setf (get mpc_msgs::TaskStatus :datatype-) "mpc_msgs/TaskStatus")
(setf (get mpc_msgs::TaskStatus :definition-)
      "uint8 task_end
")



(provide :mpc_msgs/TaskStatus "b511025659d92736cbd31617b8452ad5")


