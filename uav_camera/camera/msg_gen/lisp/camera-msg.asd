
(cl:in-package :asdf)

(defsystem "camera-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "camera_msg" :depends-on ("_package_camera_msg"))
    (:file "_package_camera_msg" :depends-on ("_package"))
  ))