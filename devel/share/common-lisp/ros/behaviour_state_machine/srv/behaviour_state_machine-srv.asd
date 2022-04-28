
(cl:in-package :asdf)

(defsystem "behaviour_state_machine-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GoalPose" :depends-on ("_package_GoalPose"))
    (:file "_package_GoalPose" :depends-on ("_package"))
  ))