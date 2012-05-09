; Auto-generated. Do not edit!


(cl:in-package uav_msgs-msg)


;//! \htmlinclude ControllerCommand.msg.html

(cl:defclass <ControllerCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (thrust
    :reader thrust
    :initarg :thrust
    :type cl:fixnum
    :initform 0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:fixnum
    :initform 0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:fixnum
    :initform 0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ControllerCommand (<ControllerCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_msgs-msg:<ControllerCommand> is deprecated: use uav_msgs-msg:ControllerCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControllerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:header-val is deprecated.  Use uav_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'thrust-val :lambda-list '(m))
(cl:defmethod thrust-val ((m <ControllerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:thrust-val is deprecated.  Use uav_msgs-msg:thrust instead.")
  (thrust m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <ControllerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:roll-val is deprecated.  Use uav_msgs-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <ControllerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:pitch-val is deprecated.  Use uav_msgs-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <ControllerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_msgs-msg:yaw-val is deprecated.  Use uav_msgs-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerCommand>) ostream)
  "Serializes a message object of type '<ControllerCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thrust)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'roll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pitch)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'yaw)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerCommand>) istream)
  "Deserializes a message object of type '<ControllerCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thrust)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'roll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pitch)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'yaw)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerCommand>)))
  "Returns string type for a message object of type '<ControllerCommand>"
  "uav_msgs/ControllerCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerCommand)))
  "Returns string type for a message object of type 'ControllerCommand"
  "uav_msgs/ControllerCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerCommand>)))
  "Returns md5sum for a message object of type '<ControllerCommand>"
  "09321cf2ffc9c3330500a4bf05bbdd35")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerCommand)))
  "Returns md5sum for a message object of type 'ControllerCommand"
  "09321cf2ffc9c3330500a4bf05bbdd35")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerCommand>)))
  "Returns full string definition for message of type '<ControllerCommand>"
  (cl:format cl:nil "Header header~%uint8 thrust~%uint8 roll~%uint8 pitch~%uint8 yaw~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerCommand)))
  "Returns full string definition for message of type 'ControllerCommand"
  (cl:format cl:nil "Header header~%uint8 thrust~%uint8 roll~%uint8 pitch~%uint8 yaw~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerCommand
    (cl:cons ':header (header msg))
    (cl:cons ':thrust (thrust msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
))
