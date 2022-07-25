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
  :slots (_task_end _task_error ))

(defmethod mpc_msgs::TaskStatus
  (:init
   (&key
    ((:task_end __task_end) 0)
    ((:task_error __task_error) 0)
    )
   (send-super :init)
   (setq _task_end (round __task_end))
   (setq _task_error (round __task_error))
   self)
  (:task_end
   (&optional __task_end)
   (if __task_end (setq _task_end __task_end)) _task_end)
  (:task_error
   (&optional __task_error)
   (if __task_error (setq _task_error __task_error)) _task_error)
  (:serialization-length
   ()
   (+
    ;; uint8 _task_end
    1
    ;; uint8 _task_error
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _task_end
       (write-byte _task_end s)
     ;; uint8 _task_error
       (write-byte _task_error s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _task_end
     (setq _task_end (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _task_error
     (setq _task_error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mpc_msgs::TaskStatus :md5sum-) "713ed9c21811428b0f7a78688cdf9573")
(setf (get mpc_msgs::TaskStatus :datatype-) "mpc_msgs/TaskStatus")
(setf (get mpc_msgs::TaskStatus :definition-)
      "uint8 task_end
uint8 task_error
")



(provide :mpc_msgs/TaskStatus "713ed9c21811428b0f7a78688cdf9573")


