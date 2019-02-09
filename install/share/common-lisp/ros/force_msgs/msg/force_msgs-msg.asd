
(cl:in-package :asdf)

(defsystem "force_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LoadCellForces32" :depends-on ("_package_LoadCellForces32"))
    (:file "_package_LoadCellForces32" :depends-on ("_package"))
  ))