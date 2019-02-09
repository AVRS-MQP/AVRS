; Auto-generated. Do not edit!


(cl:in-package coms_msgs-msg)


;//! \htmlinclude Station.msg.html

(cl:defclass <Station> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (headerstamp
    :reader headerstamp
    :initarg :headerstamp
    :type cl:real
    :initform 0)
   (xMax
    :reader xMax
    :initarg :xMax
    :type cl:float
    :initform 0.0)
   (xMin
    :reader xMin
    :initarg :xMin
    :type cl:float
    :initform 0.0)
   (yMax
    :reader yMax
    :initarg :yMax
    :type cl:float
    :initform 0.0)
   (yMin
    :reader yMin
    :initarg :yMin
    :type cl:float
    :initform 0.0)
   (zMax
    :reader zMax
    :initarg :zMax
    :type cl:float
    :initform 0.0)
   (zMin
    :reader zMin
    :initarg :zMin
    :type cl:float
    :initform 0.0)
   (flap_not_open_error
    :reader flap_not_open_error
    :initarg :flap_not_open_error
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Station (<Station>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Station>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Station)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coms_msgs-msg:<Station> is deprecated: use coms_msgs-msg:Station instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coms_msgs-msg:header-val is deprecated.  Use coms_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'headerstamp-val :lambda-list '(m))
(cl:defmethod headerstamp-val ((m <Station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coms_msgs-msg:headerstamp-val is deprecated.  Use coms_msgs-msg:headerstamp instead.")
  (headerstamp m))

(cl:ensure-generic-function 'xMax-val :lambda-list '(m))
(cl:defmethod xMax-val ((m <Station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coms_msgs-msg:xMax-val is deprecated.  Use coms_msgs-msg:xMax instead.")
  (xMax m))

(cl:ensure-generic-function 'xMin-val :lambda-list '(m))
(cl:defmethod xMin-val ((m <Station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coms_msgs-msg:xMin-val is deprecated.  Use coms_msgs-msg:xMin instead.")
  (xMin m))

(cl:ensure-generic-function 'yMax-val :lambda-list '(m))
(cl:defmethod yMax-val ((m <Station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coms_msgs-msg:yMax-val is deprecated.  Use coms_msgs-msg:yMax instead.")
  (yMax m))

(cl:ensure-generic-function 'yMin-val :lambda-list '(m))
(cl:defmethod yMin-val ((m <Station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coms_msgs-msg:yMin-val is deprecated.  Use coms_msgs-msg:yMin instead.")
  (yMin m))

(cl:ensure-generic-function 'zMax-val :lambda-list '(m))
(cl:defmethod zMax-val ((m <Station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coms_msgs-msg:zMax-val is deprecated.  Use coms_msgs-msg:zMax instead.")
  (zMax m))

(cl:ensure-generic-function 'zMin-val :lambda-list '(m))
(cl:defmethod zMin-val ((m <Station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coms_msgs-msg:zMin-val is deprecated.  Use coms_msgs-msg:zMin instead.")
  (zMin m))

(cl:ensure-generic-function 'flap_not_open_error-val :lambda-list '(m))
(cl:defmethod flap_not_open_error-val ((m <Station>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coms_msgs-msg:flap_not_open_error-val is deprecated.  Use coms_msgs-msg:flap_not_open_error instead.")
  (flap_not_open_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Station>) ostream)
  "Serializes a message object of type '<Station>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'headerstamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'headerstamp) (cl:floor (cl:slot-value msg 'headerstamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xMax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xMin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yMax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yMin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zMax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zMin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flap_not_open_error) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Station>) istream)
  "Deserializes a message object of type '<Station>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'headerstamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xMax) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xMin) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yMax) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yMin) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zMax) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zMin) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'flap_not_open_error) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Station>)))
  "Returns string type for a message object of type '<Station>"
  "coms_msgs/Station")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Station)))
  "Returns string type for a message object of type 'Station"
  "coms_msgs/Station")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Station>)))
  "Returns md5sum for a message object of type '<Station>"
  "c279ad04cbc81207a5714883b84da036")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Station)))
  "Returns md5sum for a message object of type 'Station"
  "c279ad04cbc81207a5714883b84da036")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Station>)))
  "Returns full string definition for message of type '<Station>"
  (cl:format cl:nil "Header header~%time headerstamp~%float32 xMax~%float32 xMin~%float32 yMax~%float32 yMin~%float32 zMax~%float32 zMin~%bool flap_not_open_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Station)))
  "Returns full string definition for message of type 'Station"
  (cl:format cl:nil "Header header~%time headerstamp~%float32 xMax~%float32 xMin~%float32 yMax~%float32 yMin~%float32 zMax~%float32 zMin~%bool flap_not_open_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Station>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Station>))
  "Converts a ROS message object to a list"
  (cl:list 'Station
    (cl:cons ':header (header msg))
    (cl:cons ':headerstamp (headerstamp msg))
    (cl:cons ':xMax (xMax msg))
    (cl:cons ':xMin (xMin msg))
    (cl:cons ':yMax (yMax msg))
    (cl:cons ':yMin (yMin msg))
    (cl:cons ':zMax (zMax msg))
    (cl:cons ':zMin (zMin msg))
    (cl:cons ':flap_not_open_error (flap_not_open_error msg))
))
