; Auto-generated. Do not edit!


(cl:in-package platform-msg)


;//! \htmlinclude mode_msg.msg.html

(cl:defclass <mode_msg> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass mode_msg (<mode_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mode_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mode_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name platform-msg:<mode_msg> is deprecated: use platform-msg:mode_msg instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <mode_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader platform-msg:mode-val is deprecated.  Use platform-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mode_msg>) ostream)
  "Serializes a message object of type '<mode_msg>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mode_msg>) istream)
  "Deserializes a message object of type '<mode_msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mode_msg>)))
  "Returns string type for a message object of type '<mode_msg>"
  "platform/mode_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mode_msg)))
  "Returns string type for a message object of type 'mode_msg"
  "platform/mode_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mode_msg>)))
  "Returns md5sum for a message object of type '<mode_msg>"
  "ff63f6ea3c3e9b7504b2cb3ee0a09d92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mode_msg)))
  "Returns md5sum for a message object of type 'mode_msg"
  "ff63f6ea3c3e9b7504b2cb3ee0a09d92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mode_msg>)))
  "Returns full string definition for message of type '<mode_msg>"
  (cl:format cl:nil "int32 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mode_msg)))
  "Returns full string definition for message of type 'mode_msg"
  (cl:format cl:nil "int32 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mode_msg>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mode_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'mode_msg
    (cl:cons ':mode (mode msg))
))
