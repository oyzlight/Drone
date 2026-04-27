;; Auto-generated. Do not edit!


(when (boundp 't1_offboard_takeoff::Mydata)
  (if (not (find-package "T1_OFFBOARD_TAKEOFF"))
    (make-package "T1_OFFBOARD_TAKEOFF"))
  (shadow 'Mydata (find-package "T1_OFFBOARD_TAKEOFF")))
(unless (find-package "T1_OFFBOARD_TAKEOFF::MYDATA")
  (make-package "T1_OFFBOARD_TAKEOFF::MYDATA"))

(in-package "ROS")
;;//! \htmlinclude Mydata.msg.html


(defclass t1_offboard_takeoff::Mydata
  :super ros::object
  :slots (_data1 _data2 ))

(defmethod t1_offboard_takeoff::Mydata
  (:init
   (&key
    ((:data1 __data1) 0)
    ((:data2 __data2) 0)
    )
   (send-super :init)
   (setq _data1 (round __data1))
   (setq _data2 (round __data2))
   self)
  (:data1
   (&optional __data1)
   (if __data1 (setq _data1 __data1)) _data1)
  (:data2
   (&optional __data2)
   (if __data2 (setq _data2 __data2)) _data2)
  (:serialization-length
   ()
   (+
    ;; int16 _data1
    2
    ;; int16 _data2
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _data1
       (write-word _data1 s)
     ;; int16 _data2
       (write-word _data2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _data1
     (setq _data1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _data2
     (setq _data2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get t1_offboard_takeoff::Mydata :md5sum-) "0e8db631b857a5e6de39b526533f57ee")
(setf (get t1_offboard_takeoff::Mydata :datatype-) "t1_offboard_takeoff/Mydata")
(setf (get t1_offboard_takeoff::Mydata :definition-)
      "int16 data1
int16 data2

")



(provide :t1_offboard_takeoff/Mydata "0e8db631b857a5e6de39b526533f57ee")


