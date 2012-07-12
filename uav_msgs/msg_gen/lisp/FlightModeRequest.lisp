; Auto-generated. Do not edit!


(cl:in-package uav_msgs-msg)


;//! \htmlinclude FlightModeRequest.msg.html

(cl:defclass <FlightModeRequest> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass FlightModeRequest (<FlightModeRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlightModeRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlightModeRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_msgs-msg:<FlightModeRequest> is deprecated: use uav_msgs-msg:FlightModeRequest instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <FlightModeRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:mode-val is deprecated.  Use uav_msgs-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FlightModeRequest>)))
    "Constants for message type '<FlightModeRequest>"
  '((:NONE . 0)
    (:LAND . 1)
    (:TAKE_OFF . 2)
    (:HOVER . 3)
    (:FOLLOW . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FlightModeRequest)))
    "Constants for message type 'FlightModeRequest"
  '((:NONE . 0)
    (:LAND . 1)
    (:TAKE_OFF . 2)
    (:HOVER . 3)
    (:FOLLOW . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlightModeRequest>) ostream)
  "Serializes a message object of type '<FlightModeRequest>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlightModeRequest>) istream)
  "Deserializes a message object of type '<FlightModeRequest>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlightModeRequest>)))
  "Returns string type for a message object of type '<FlightModeRequest>"
  "uav_msgs/FlightModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlightModeRequest)))
  "Returns string type for a message object of type 'FlightModeRequest"
  "uav_msgs/FlightModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlightModeRequest>)))
  "Returns md5sum for a message object of type '<FlightModeRequest>"
  "07471b540fad40d391881a5e198b67c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlightModeRequest)))
  "Returns md5sum for a message object of type 'FlightModeRequest"
  "07471b540fad40d391881a5e198b67c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlightModeRequest>)))
  "Returns full string definition for message of type '<FlightModeRequest>"
  (cl:format cl:nil "byte NONE=0~%byte LAND=1~%byte TAKE_OFF=2~%byte HOVER=3~%byte FOLLOW=4~%byte mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlightModeRequest)))
  "Returns full string definition for message of type 'FlightModeRequest"
  (cl:format cl:nil "byte NONE=0~%byte LAND=1~%byte TAKE_OFF=2~%byte HOVER=3~%byte FOLLOW=4~%byte mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlightModeRequest>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlightModeRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'FlightModeRequest
    (cl:cons ':mode (mode msg))
))
