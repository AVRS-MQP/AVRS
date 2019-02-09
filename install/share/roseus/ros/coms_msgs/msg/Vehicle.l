;; Auto-generated. Do not edit!


(when (boundp 'coms_msgs::Vehicle)
  (if (not (find-package "COMS_MSGS"))
    (make-package "COMS_MSGS"))
  (shadow 'Vehicle (find-package "COMS_MSGS")))
(unless (find-package "COMS_MSGS::VEHICLE")
  (make-package "COMS_MSGS::VEHICLE"))

(in-package "ROS")
;;//! \htmlinclude Vehicle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass coms_msgs::Vehicle
  :super ros::object
  :slots (_header _headerstamp _model _charger_type _battery_charge _charge_level _flap_unlocked _flap_auto_open ))

(defmethod coms_msgs::Vehicle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:headerstamp __headerstamp) (instance ros::time :init))
    ((:model __model) "")
    ((:charger_type __charger_type) "")
    ((:battery_charge __battery_charge) 0.0)
    ((:charge_level __charge_level) 0)
    ((:flap_unlocked __flap_unlocked) nil)
    ((:flap_auto_open __flap_auto_open) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _headerstamp __headerstamp)
   (setq _model (string __model))
   (setq _charger_type (string __charger_type))
   (setq _battery_charge (float __battery_charge))
   (setq _charge_level (round __charge_level))
   (setq _flap_unlocked __flap_unlocked)
   (setq _flap_auto_open __flap_auto_open)
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
  (:model
   (&optional __model)
   (if __model (setq _model __model)) _model)
  (:charger_type
   (&optional __charger_type)
   (if __charger_type (setq _charger_type __charger_type)) _charger_type)
  (:battery_charge
   (&optional __battery_charge)
   (if __battery_charge (setq _battery_charge __battery_charge)) _battery_charge)
  (:charge_level
   (&optional __charge_level)
   (if __charge_level (setq _charge_level __charge_level)) _charge_level)
  (:flap_unlocked
   (&optional __flap_unlocked)
   (if __flap_unlocked (setq _flap_unlocked __flap_unlocked)) _flap_unlocked)
  (:flap_auto_open
   (&optional __flap_auto_open)
   (if __flap_auto_open (setq _flap_auto_open __flap_auto_open)) _flap_auto_open)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _headerstamp
    8
    ;; string _model
    4 (length _model)
    ;; string _charger_type
    4 (length _charger_type)
    ;; float32 _battery_charge
    4
    ;; int32 _charge_level
    4
    ;; bool _flap_unlocked
    1
    ;; bool _flap_auto_open
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
     ;; string _model
       (write-long (length _model) s) (princ _model s)
     ;; string _charger_type
       (write-long (length _charger_type) s) (princ _charger_type s)
     ;; float32 _battery_charge
       (sys::poke _battery_charge (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _charge_level
       (write-long _charge_level s)
     ;; bool _flap_unlocked
       (if _flap_unlocked (write-byte -1 s) (write-byte 0 s))
     ;; bool _flap_auto_open
       (if _flap_auto_open (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; time _headerstamp
     (send _headerstamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _headerstamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _model
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _charger_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _charger_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _battery_charge
     (setq _battery_charge (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _charge_level
     (setq _charge_level (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _flap_unlocked
     (setq _flap_unlocked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _flap_auto_open
     (setq _flap_auto_open (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get coms_msgs::Vehicle :md5sum-) "15c570864c534a992bcad74ac9993ed9")
(setf (get coms_msgs::Vehicle :datatype-) "coms_msgs/Vehicle")
(setf (get coms_msgs::Vehicle :definition-)
      "Header header
time headerstamp
string model
string charger_type
float32 battery_charge
int32 charge_level
bool flap_unlocked
bool flap_auto_open

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



(provide :coms_msgs/Vehicle "15c570864c534a992bcad74ac9993ed9")


