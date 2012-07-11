
(cl:in-package :asdf)

(defsystem "platform-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "mode_msg" :depends-on ("_package_mode_msg"))
    (:file "_package_mode_msg" :depends-on ("_package"))
  ))