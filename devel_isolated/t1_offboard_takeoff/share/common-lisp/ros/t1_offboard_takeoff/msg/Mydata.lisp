; Auto-generated. Do not edit!


(cl:in-package t1_offboard_takeoff-msg)


;//! \htmlinclude Mydata.msg.html

(cl:defclass <Mydata> (roslisp-msg-protocol:ros-message)
  ((data1
    :reader data1
    :initarg :data1
    :type cl:fixnum
    :initform 0)
   (data2
    :reader data2
    :initarg :data2
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Mydata (<Mydata>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Mydata>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Mydata)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name t1_offboard_takeoff-msg:<Mydata> is deprecated: use t1_offboard_takeoff-msg:Mydata instead.")))

(cl:ensure-generic-function 'data1-val :lambda-list '(m))
(cl:defmethod data1-val ((m <Mydata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader t1_offboard_takeoff-msg:data1-val is deprecated.  Use t1_offboard_takeoff-msg:data1 instead.")
  (data1 m))

(cl:ensure-generic-function 'data2-val :lambda-list '(m))
(cl:defmethod data2-val ((m <Mydata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader t1_offboard_takeoff-msg:data2-val is deprecated.  Use t1_offboard_takeoff-msg:data2 instead.")
  (data2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Mydata>) ostream)
  "Serializes a message object of type '<Mydata>"
  (cl:let* ((signed (cl:slot-value msg 'data1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'data2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Mydata>) istream)
  "Deserializes a message object of type '<Mydata>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data1) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data2) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Mydata>)))
  "Returns string type for a message object of type '<Mydata>"
  "t1_offboard_takeoff/Mydata")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Mydata)))
  "Returns string type for a message object of type 'Mydata"
  "t1_offboard_takeoff/Mydata")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Mydata>)))
  "Returns md5sum for a message object of type '<Mydata>"
  "0e8db631b857a5e6de39b526533f57ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Mydata)))
  "Returns md5sum for a message object of type 'Mydata"
  "0e8db631b857a5e6de39b526533f57ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Mydata>)))
  "Returns full string definition for message of type '<Mydata>"
  (cl:format cl:nil "int16 data1~%int16 data2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Mydata)))
  "Returns full string definition for message of type 'Mydata"
  (cl:format cl:nil "int16 data1~%int16 data2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Mydata>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Mydata>))
  "Converts a ROS message object to a list"
  (cl:list 'Mydata
    (cl:cons ':data1 (data1 msg))
    (cl:cons ':data2 (data2 msg))
))
