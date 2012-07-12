; Auto-generated. Do not edit!


(cl:in-package uav_msgs-msg)


;//! \htmlinclude camera_msg.msg.html

(cl:defclass <camera_msg> (roslisp-msg-protocol:ros-message)
  ((change_res
    :reader change_res
    :initarg :change_res
    :type cl:integer
    :initform 0))
)

(cl:defclass camera_msg (<camera_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <camera_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'camera_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_msgs-msg:<camera_msg> is deprecated: use uav_msgs-msg:camera_msg instead.")))

(cl:ensure-generic-function 'change_res-val :lambda-list '(m))
(cl:defmethod change_res-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:change_res-val is deprecated.  Use uav_msgs-msg:change_res instead.")
  (change_res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <camera_msg>) ostream)
  "Serializes a message object of type '<camera_msg>"
  (cl:let* ((signed (cl:slot-value msg 'change_res)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <camera_msg>) istream)
  "Deserializes a message object of type '<camera_msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'change_res) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<camera_msg>)))
  "Returns string type for a message object of type '<camera_msg>"
  "uav_msgs/camera_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'camera_msg)))
  "Returns string type for a message object of type 'camera_msg"
  "uav_msgs/camera_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<camera_msg>)))
  "Returns md5sum for a message object of type '<camera_msg>"
  "c7d90e90999ece5a54c013134180c080")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'camera_msg)))
  "Returns md5sum for a message object of type 'camera_msg"
  "c7d90e90999ece5a54c013134180c080")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<camera_msg>)))
  "Returns full string definition for message of type '<camera_msg>"
  (cl:format cl:nil "int32 change_res~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'camera_msg)))
  "Returns full string definition for message of type 'camera_msg"
  (cl:format cl:nil "int32 change_res~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <camera_msg>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <camera_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'camera_msg
    (cl:cons ':change_res (change_res msg))
))
