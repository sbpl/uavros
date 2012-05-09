
(cl:in-package :asdf)

(defsystem "uav_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ControllerCommand" :depends-on ("_package_ControllerCommand"))
    (:file "_package_ControllerCommand" :depends-on ("_package"))
  ))