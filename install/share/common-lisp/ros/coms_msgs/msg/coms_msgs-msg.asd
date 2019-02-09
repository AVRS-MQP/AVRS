
(cl:in-package :asdf)

(defsystem "coms_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Station" :depends-on ("_package_Station"))
    (:file "_package_Station" :depends-on ("_package"))
    (:file "Vehicle" :depends-on ("_package_Vehicle"))
    (:file "_package_Vehicle" :depends-on ("_package"))
  ))