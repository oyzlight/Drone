
(cl:in-package :asdf)

(defsystem "t1_offboard_takeoff-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Mydata" :depends-on ("_package_Mydata"))
    (:file "_package_Mydata" :depends-on ("_package"))
  ))