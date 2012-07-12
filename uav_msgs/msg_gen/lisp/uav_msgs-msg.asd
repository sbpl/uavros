
(cl:in-package :asdf)

(defsystem "uav_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ControllerCommand" :depends-on ("_package_ControllerCommand"))
    (:file "_package_ControllerCommand" :depends-on ("_package"))
    (:file "FlightModeStatus" :depends-on ("_package_FlightModeStatus"))
    (:file "_package_FlightModeStatus" :depends-on ("_package"))
    (:file "camera_msg" :depends-on ("_package_camera_msg"))
    (:file "_package_camera_msg" :depends-on ("_package"))
    (:file "FlightModeRequest" :depends-on ("_package_FlightModeRequest"))
    (:file "_package_FlightModeRequest" :depends-on ("_package"))
    (:file "mode_msg" :depends-on ("_package_mode_msg"))
    (:file "_package_mode_msg" :depends-on ("_package"))
  ))