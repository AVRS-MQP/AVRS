; Auto-generated. Do not edit!


(cl:in-package force_msgs-msg)


;//! \htmlinclude LoadCellForces32.msg.html

(cl:defclass <LoadCellForces32> (roslisp-msg-protocol:ros-message)
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
   (cellA
    :reader cellA
    :initarg :cellA
    :type cl:float
    :initform 0.0)
   (cellB
    :reader cellB
    :initarg :cellB
    :type cl:float
    :initform 0.0)
   (cellC
    :reader cellC
    :initarg :cellC
    :type cl:float
    :initform 0.0))
)

(cl:defclass LoadCellForces32 (<LoadCellForces32>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadCellForces32>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadCellForces32)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name force_msgs-msg:<LoadCellForces32> is deprecated: use force_msgs-msg:LoadCellForces32 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LoadCellForces32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader force_msgs-msg:header-val is deprecated.  Use force_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'headerstamp-val :lambda-list '(m))
(cl:defmethod headerstamp-val ((m <LoadCellForces32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader force_msgs-msg:headerstamp-val is deprecated.  Use force_msgs-msg:headerstamp instead.")
  (headerstamp m))

(cl:ensure-generic-function 'cellA-val :lambda-list '(m))
(cl:defmethod cellA-val ((m <LoadCellForces32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader force_msgs-msg:cellA-val is deprecated.  Use force_msgs-msg:cellA instead.")
  (cellA m))

(cl:ensure-generic-function 'cellB-val :lambda-list '(m))
(cl:defmethod cellB-val ((m <LoadCellForces32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader force_msgs-msg:cellB-val is deprecated.  Use force_msgs-msg:cellB instead.")
  (cellB m))

(cl:ensure-generic-function 'cellC-val :lambda-list '(m))
(cl:defmethod cellC-val ((m <LoadCellForces32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader force_msgs-msg:cellC-val is deprecated.  Use force_msgs-msg:cellC instead.")
  (cellC m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadCellForces32>) ostream)
  "Serializes a message object of type '<LoadCellForces32>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cellA))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cellB))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cellC))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadCellForces32>) istream)
  "Deserializes a message object of type '<LoadCellForces32>"
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
    (cl:setf (cl:slot-value msg 'cellA) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cellB) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cellC) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadCellForces32>)))
  "Returns string type for a message object of type '<LoadCellForces32>"
  "force_msgs/LoadCellForces32")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadCellForces32)))
  "Returns string type for a message object of type 'LoadCellForces32"
  "force_msgs/LoadCellForces32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadCellForces32>)))
  "Returns md5sum for a message object of type '<LoadCellForces32>"
  "f212bc1b7495a187d9c991f6bd06e446")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadCellForces32)))
  "Returns md5sum for a message object of type 'LoadCellForces32"
  "f212bc1b7495a187d9c991f6bd06e446")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadCellForces32>)))
  "Returns full string definition for message of type '<LoadCellForces32>"
  (cl:format cl:nil "Header header~%time headerstamp~%float32 cellA~%float32 cellB~%float32 cellC~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadCellForces32)))
  "Returns full string definition for message of type 'LoadCellForces32"
  (cl:format cl:nil "Header header~%time headerstamp~%float32 cellA~%float32 cellB~%float32 cellC~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadCellForces32>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadCellForces32>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadCellForces32
    (cl:cons ':header (header msg))
    (cl:cons ':headerstamp (headerstamp msg))
    (cl:cons ':cellA (cellA msg))
    (cl:cons ':cellB (cellB msg))
    (cl:cons ':cellC (cellC msg))
))
