;; Auto-generated. Do not edit!


(when (boundp 'mpc_msgs::VehicleStatus)
  (if (not (find-package "MPC_MSGS"))
    (make-package "MPC_MSGS"))
  (shadow 'VehicleStatus (find-package "MPC_MSGS")))
(unless (find-package "MPC_MSGS::VEHICLESTATUS")
  (make-package "MPC_MSGS::VEHICLESTATUS"))

(in-package "ROS")
;;//! \htmlinclude VehicleStatus.msg.html


(defclass mpc_msgs::VehicleStatus
  :super ros::object
  :slots (_car_mode _error_level _door_state _lamp_L _lamp_R _voltage _current _steer _hand_brake _gear _stop _acc _speed ))

(defmethod mpc_msgs::VehicleStatus
  (:init
   (&key
    ((:car_mode __car_mode) 0)
    ((:error_level __error_level) 0)
    ((:door_state __door_state) 0)
    ((:lamp_L __lamp_L) 0)
    ((:lamp_R __lamp_R) 0)
    ((:voltage __voltage) 0.0)
    ((:current __current) 0.0)
    ((:steer __steer) 0.0)
    ((:hand_brake __hand_brake) 0)
    ((:gear __gear) 0)
    ((:stop __stop) 0)
    ((:acc __acc) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _car_mode (round __car_mode))
   (setq _error_level (round __error_level))
   (setq _door_state (round __door_state))
   (setq _lamp_L (round __lamp_L))
   (setq _lamp_R (round __lamp_R))
   (setq _voltage (float __voltage))
   (setq _current (float __current))
   (setq _steer (float __steer))
   (setq _hand_brake (round __hand_brake))
   (setq _gear (round __gear))
   (setq _stop (round __stop))
   (setq _acc (float __acc))
   (setq _speed (float __speed))
   self)
  (:car_mode
   (&optional __car_mode)
   (if __car_mode (setq _car_mode __car_mode)) _car_mode)
  (:error_level
   (&optional __error_level)
   (if __error_level (setq _error_level __error_level)) _error_level)
  (:door_state
   (&optional __door_state)
   (if __door_state (setq _door_state __door_state)) _door_state)
  (:lamp_L
   (&optional __lamp_L)
   (if __lamp_L (setq _lamp_L __lamp_L)) _lamp_L)
  (:lamp_R
   (&optional __lamp_R)
   (if __lamp_R (setq _lamp_R __lamp_R)) _lamp_R)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:steer
   (&optional __steer)
   (if __steer (setq _steer __steer)) _steer)
  (:hand_brake
   (&optional __hand_brake)
   (if __hand_brake (setq _hand_brake __hand_brake)) _hand_brake)
  (:gear
   (&optional __gear)
   (if __gear (setq _gear __gear)) _gear)
  (:stop
   (&optional __stop)
   (if __stop (setq _stop __stop)) _stop)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; uint8 _car_mode
    1
    ;; uint8 _error_level
    1
    ;; uint8 _door_state
    1
    ;; uint8 _lamp_L
    1
    ;; uint8 _lamp_R
    1
    ;; float32 _voltage
    4
    ;; float32 _current
    4
    ;; float32 _steer
    4
    ;; uint8 _hand_brake
    1
    ;; uint8 _gear
    1
    ;; uint8 _stop
    1
    ;; float32 _acc
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _car_mode
       (write-byte _car_mode s)
     ;; uint8 _error_level
       (write-byte _error_level s)
     ;; uint8 _door_state
       (write-byte _door_state s)
     ;; uint8 _lamp_L
       (write-byte _lamp_L s)
     ;; uint8 _lamp_R
       (write-byte _lamp_R s)
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current
       (sys::poke _current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steer
       (sys::poke _steer (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _hand_brake
       (write-byte _hand_brake s)
     ;; uint8 _gear
       (write-byte _gear s)
     ;; uint8 _stop
       (write-byte _stop s)
     ;; float32 _acc
       (sys::poke _acc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _car_mode
     (setq _car_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _error_level
     (setq _error_level (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _door_state
     (setq _door_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _lamp_L
     (setq _lamp_L (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _lamp_R
     (setq _lamp_R (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current
     (setq _current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steer
     (setq _steer (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _hand_brake
     (setq _hand_brake (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gear
     (setq _gear (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _stop
     (setq _stop (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _acc
     (setq _acc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mpc_msgs::VehicleStatus :md5sum-) "b163f7630993eb80c9c2c58fe427804d")
(setf (get mpc_msgs::VehicleStatus :datatype-) "mpc_msgs/VehicleStatus")
(setf (get mpc_msgs::VehicleStatus :definition-)
      "uint8 car_mode
uint8 error_level
uint8 door_state
uint8 lamp_L
uint8 lamp_R
float32 voltage
float32 current
float32 steer
uint8 hand_brake
uint8 gear
uint8 stop
float32 acc
float32 speed

")



(provide :mpc_msgs/VehicleStatus "b163f7630993eb80c9c2c58fe427804d")


