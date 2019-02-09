;; Auto-generated. Do not edit!


(when (boundp 'coms_msgs::Station)
  (if (not (find-package "COMS_MSGS"))
    (make-package "COMS_MSGS"))
  (shadow 'Station (find-package "COMS_MSGS")))
(unless (find-package "COMS_MSGS::STATION")
  (make-package "COMS_MSGS::STATION"))

(in-package "ROS")
;;//! \htmlinclude Station.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass coms_msgs::Station
  :super ros::object
  :slots (_header _headerstamp _xMax _xMin _yMax _yMin _zMax _zMin _flap_not_open_error ))

(defmethod coms_msgs::Station
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:headerstamp __headerstamp) (instance ros::time :init))
    ((:xMax __xMax) 0.0)
    ((:xMin __xMin) 0.0)
    ((:yMax __yMax) 0.0)
    ((:yMin __yMin) 0.0)
    ((:zMax __zMax) 0.0)
    ((:zMin __zMin) 0.0)
    ((:flap_not_open_error __flap_not_open_error) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _headerstamp __headerstamp)
   (setq _xMax (float __xMax))
   (setq _xMin (float __xMin))
   (setq _yMax (float __yMax))
   (setq _yMin (float __yMin))
   (setq _zMax (float __zMax))
   (setq _zMin (float __zMin))
   (setq _flap_not_open_error __flap_not_open_error)
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
  (:xMax
   (&optional __xMax)
   (if __xMax (setq _xMax __xMax)) _xMax)
  (:xMin
   (&optional __xMin)
   (if __xMin (setq _xMin __xMin)) _xMin)
  (:yMax
   (&optional __yMax)
   (if __yMax (setq _yMax __yMax)) _yMax)
  (:yMin
   (&optional __yMin)
   (if __yMin (setq _yMin __yMin)) _yMin)
  (:zMax
   (&optional __zMax)
   (if __zMax (setq _zMax __zMax)) _zMax)
  (:zMin
   (&optional __zMin)
   (if __zMin (setq _zMin __zMin)) _zMin)
  (:flap_not_open_error
   (&optional __flap_not_open_error)
   (if __flap_not_open_error (setq _flap_not_open_error __flap_not_open_error)) _flap_not_open_error)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _headerstamp
    8
    ;; float32 _xMax
    4
    ;; float32 _xMin
    4
    ;; float32 _yMax
    4
    ;; float32 _yMin
    4
    ;; float32 _zMax
    4
    ;; float32 _zMin
    4
    ;; bool _flap_not_open_error
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; time _headerstamp
       (write-long (send _headerstamp :sec) s) (write-long (send _headerstamp :nsec) s)
     ;; float32 _xMax
       (sys::poke _xMax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _xMin
       (sys::poke _xMin (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yMax
       (sys::poke _yMax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yMin
       (sys::poke _yMin (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _zMax
       (sys::poke _zMax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _zMin
       (sys::poke _zMin (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _flap_not_open_error
       (if _flap_not_open_error (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; time _headerstamp
     (send _headerstamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _headerstamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _xMax
     (setq _xMax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _xMin
     (setq _xMin (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yMax
     (setq _yMax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yMin
     (setq _yMin (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _zMax
     (setq _zMax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _zMin
     (setq _zMin (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _flap_not_open_error
     (setq _flap_not_open_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get coms_msgs::Station :md5sum-) "c279ad04cbc81207a5714883b84da036")
(setf (get coms_msgs::Station :datatype-) "coms_msgs/Station")
(setf (get coms_msgs::Station :definition-)
      "Header header
time headerstamp
float32 xMax
float32 xMin
float32 yMax
float32 yMin
float32 zMax
float32 zMin
bool flap_not_open_error

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



(provide :coms_msgs/Station "c279ad04cbc81207a5714883b84da036")


