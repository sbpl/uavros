; Auto-generated. Do not edit!


(cl:in-package uav_msgs-msg)


;//! \htmlinclude FlightModeStatus.msg.html

(cl:defclass <FlightModeStatus> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass FlightModeStatus (<FlightModeStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlightModeStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlightModeStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_msgs-msg:<FlightModeStatus> is deprecated: use uav_msgs-msg:FlightModeStatus instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <FlightModeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:mode-val is deprecated.  Use uav_msgs-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FlightModeStatus>)))
    "Constants for message type '<FlightModeStatus>"
  '((:LANDED . 0)
    (:LANDING . 1)
    (:TAKE_OFF . 2)
    (:HOVER . 3)
    (:FOLLOWING . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FlightModeStatus)))
    "Constants for message type 'FlightModeStatus"
  '((:LANDED . 0)
    (:LANDING . 1)
    (:TAKE_OFF . 2)
    (:HOVER . 3)
    (:FOLLOWING . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlightModeStatus>) ostream)
  "Serializes a message object of type '<FlightModeStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlightModeStatus>) istream)
  "Deserializes a message object of type '<FlightModeStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlightModeStatus>)))
  "Returns string type for a message object of type '<FlightModeStatus>"
  "uav_msgs/FlightModeStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlightModeStatus)))
  "Returns string type for a message object of type 'FlightModeStatus"
  "uav_msgs/FlightModeStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlightModeStatus>)))
  "Returns md5sum for a message object of type '<FlightModeStatus>"
  "5a8c5566e2b0a74c06a40b5b88e4ef8f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlightModeStatus)))
  "Returns md5sum for a message object of type 'FlightModeStatus"
  "5a8c5566e2b0a74c06a40b5b88e4ef8f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlightModeStatus>)))
  "Returns full string definition for message of type '<FlightModeStatus>"
  (cl:format cl:nil "byte LANDED=0~%byte LANDING=1~%byte TAKE_OFF=2~%byte HOVER=3~%byte FOLLOWING=4~%byte mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlightModeStatus)))
  "Returns full string definition for message of type 'FlightModeStatus"
  (cl:format cl:nil "byte LANDED=0~%byte LANDING=1~%byte TAKE_OFF=2~%byte HOVER=3~%byte FOLLOWING=4~%byte mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlightModeStatus>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlightModeStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'FlightModeStatus
    (cl:cons ':mode (mode msg))
))
