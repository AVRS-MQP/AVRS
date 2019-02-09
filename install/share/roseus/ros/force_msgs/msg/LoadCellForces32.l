;; Auto-generated. Do not edit!


(when (boundp 'force_msgs::LoadCellForces32)
  (if (not (find-package "FORCE_MSGS"))
    (make-package "FORCE_MSGS"))
  (shadow 'LoadCellForces32 (find-package "FORCE_MSGS")))
(unless (find-package "FORCE_MSGS::LOADCELLFORCES32")
  (make-package "FORCE_MSGS::LOADCELLFORCES32"))

(in-package "ROS")
;;//! \htmlinclude LoadCellForces32.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass force_msgs::LoadCellForces32
  :super ros::object
  :slots (_header _headerstamp _cellA _cellB _cellC ))

(defmethod force_msgs::LoadCellForces32
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:headerstamp __headerstamp) (instance ros::time :init))
    ((:cellA __cellA) 0.0)
    ((:cellB __cellB) 0.0)
    ((:cellC __cellC) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _headerstamp __headerstamp)
   (setq _cellA (float __cellA))
   (setq _cellB (float __cellB))
   (setq _cellC (float __cellC))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:headerstamp
   (&optional __headerstamp)
   (if __headerstamp (setq _headerstamp __headerstamp)) _headerstamp)
  (:cellA
   (&optional __cellA)
   (if __cellA (setq _cellA __cellA)) _cellA)
  (:cellB
   (&optional __cellB)
   (if __cellB (setq _cellB __cellB)) _cellB)
  (:cellC
   (&optional __cellC)
   (if __cellC (setq _cellC __cellC)) _cellC)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _headerstamp
    8
    ;; float32 _cellA
    4
    ;; float32 _cellB
    4
    ;; float32 _cellC
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; time _headerstamp
       (write-long (send _headerstamp :sec) s) (write-long (send _headerstamp :nsec) s)
     ;; float32 _cellA
       (sys::poke _cellA (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cellB
       (sys::poke _cellB (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cellC
       (sys::poke _cellC (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; time _headerstamp
     (send _headerstamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _headerstamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _cellA
     (setq _cellA (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cellB
     (setq _cellB (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cellC
     (setq _cellC (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get force_msgs::LoadCellForces32 :md5sum-) "f212bc1b7495a187d9c991f6bd06e446")
(setf (get force_msgs::LoadCellForces32 :datatype-) "force_msgs/LoadCellForces32")
(setf (get force_msgs::LoadCellForces32 :definition-)
      "Header header
time headerstamp
float32 cellA
float32 cellB
float32 cellC


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :force_msgs/LoadCellForces32 "f212bc1b7495a187d9c991f6bd06e446")


