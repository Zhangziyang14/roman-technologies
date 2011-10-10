; Auto-generated. Do not edit!


(cl:in-package mobile_base-msg)


;//! \htmlinclude tweak.msg.html

(cl:defclass <tweak> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0)
   (motorID
    :reader motorID
    :initarg :motorID
    :type cl:integer
    :initform 0))
)

(cl:defclass tweak (<tweak>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tweak>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tweak)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobile_base-msg:<tweak> is deprecated: use mobile_base-msg:tweak instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <tweak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobile_base-msg:data-val is deprecated.  Use mobile_base-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'motorID-val :lambda-list '(m))
(cl:defmethod motorID-val ((m <tweak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobile_base-msg:motorID-val is deprecated.  Use mobile_base-msg:motorID instead.")
  (motorID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tweak>) ostream)
  "Serializes a message object of type '<tweak>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motorID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tweak>) istream)
  "Deserializes a message object of type '<tweak>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motorID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tweak>)))
  "Returns string type for a message object of type '<tweak>"
  "mobile_base/tweak")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tweak)))
  "Returns string type for a message object of type 'tweak"
  "mobile_base/tweak")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tweak>)))
  "Returns md5sum for a message object of type '<tweak>"
  "2f63c8c19490acbf4d6d60c299883f03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tweak)))
  "Returns md5sum for a message object of type 'tweak"
  "2f63c8c19490acbf4d6d60c299883f03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tweak>)))
  "Returns full string definition for message of type '<tweak>"
  (cl:format cl:nil "int32 data~%int32 motorID~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tweak)))
  "Returns full string definition for message of type 'tweak"
  (cl:format cl:nil "int32 data~%int32 motorID~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tweak>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tweak>))
  "Converts a ROS message object to a list"
  (cl:list 'tweak
    (cl:cons ':data (data msg))
    (cl:cons ':motorID (motorID msg))
))
