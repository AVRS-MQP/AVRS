;; Auto-generated. Do not edit!


(when (boundp 'mode_msgs::Mode)
  (if (not (find-package "MODE_MSGS"))
    (make-package "MODE_MSGS"))
  (shadow 'Mode (find-package "MODE_MSGS")))
(unless (find-package "MODE_MSGS::MODE")
  (make-package "MODE_MSGS::MODE"))
(unless (find-package "MODE_MSGS::MODEREQUEST")
  (make-package "MODE_MSGS::MODEREQUEST"))
(unless (find-package "MODE_MSGS::MODERESPONSE")
  (make-package "MODE_MSGS::MODERESPONSE"))

(in-package "ROS")





(defclass mode_msgs::ModeRequest
  :super ros::object
  :slots (_mode _input2 ))

(defmethod mode_msgs::ModeRequest
  (:init
   (&key
    ((:mode __mode) "")
    ((:input2 __input2) 0)
    )
   (send-super :init)
   (setq _mode (string __mode))
   (setq _input2 (round __input2))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:input2
   (&optional __input2)
   (if __input2 (setq _input2 __input2)) _input2)
  (:serialization-length
   ()
   (+
    ;; string _mode
    4 (length _mode)
    ;; int32 _input2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; int32 _input2
       (write-long _input2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _input2
     (setq _input2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mode_msgs::ModeResponse
  :super ros::object
  :slots (_output _done ))

(defmethod mode_msgs::ModeResponse
  (:init
   (&key
    ((:output __output) "")
    ((:done __done) nil)
    )
   (send-super :init)
   (setq _output (string __output))
   (setq _done __done)
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:done
   (&optional __done)
   (if __done (setq _done __done)) _done)
  (:serialization-length
   ()
   (+
    ;; string _output
    4 (length _output)
    ;; bool _done
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _output
       (write-long (length _output) s) (princ _output s)
     ;; bool _done
       (if _done (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _output
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _output (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _done
     (setq _done (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass mode_msgs::Mode
  :super ros::object
  :slots ())

(setf (get mode_msgs::Mode :md5sum-) "54307bda67f170e4b3f87373deb748d3")
(setf (get mode_msgs::Mode :datatype-) "mode_msgs/Mode")
(setf (get mode_msgs::Mode :request) mode_msgs::ModeRequest)
(setf (get mode_msgs::Mode :response) mode_msgs::ModeResponse)

(defmethod mode_msgs::ModeRequest
  (:response () (instance mode_msgs::ModeResponse :init)))

(setf (get mode_msgs::ModeRequest :md5sum-) "54307bda67f170e4b3f87373deb748d3")
(setf (get mode_msgs::ModeRequest :datatype-) "mode_msgs/ModeRequest")
(setf (get mode_msgs::ModeRequest :definition-)
      "string mode
int32 input2
---
string output
bool done


")

(setf (get mode_msgs::ModeResponse :md5sum-) "54307bda67f170e4b3f87373deb748d3")
(setf (get mode_msgs::ModeResponse :datatype-) "mode_msgs/ModeResponse")
(setf (get mode_msgs::ModeResponse :definition-)
      "string mode
int32 input2
---
string output
bool done


")



(provide :mode_msgs/Mode "54307bda67f170e4b3f87373deb748d3")


