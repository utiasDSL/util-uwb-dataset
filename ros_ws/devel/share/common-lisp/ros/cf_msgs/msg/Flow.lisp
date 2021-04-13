; Auto-generated. Do not edit!


(cl:in-package cf_msgs-msg)


;//! \htmlinclude Flow.msg.html

(cl:defclass <Flow> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (deltaX
    :reader deltaX
    :initarg :deltaX
    :type cl:integer
    :initform 0)
   (deltaY
    :reader deltaY
    :initarg :deltaY
    :type cl:integer
    :initform 0))
)

(cl:defclass Flow (<Flow>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Flow>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Flow)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cf_msgs-msg:<Flow> is deprecated: use cf_msgs-msg:Flow instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Flow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:header-val is deprecated.  Use cf_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'deltaX-val :lambda-list '(m))
(cl:defmethod deltaX-val ((m <Flow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:deltaX-val is deprecated.  Use cf_msgs-msg:deltaX instead.")
  (deltaX m))

(cl:ensure-generic-function 'deltaY-val :lambda-list '(m))
(cl:defmethod deltaY-val ((m <Flow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:deltaY-val is deprecated.  Use cf_msgs-msg:deltaY instead.")
  (deltaY m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Flow>) ostream)
  "Serializes a message object of type '<Flow>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'deltaX)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'deltaY)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Flow>) istream)
  "Deserializes a message object of type '<Flow>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'deltaX) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'deltaY) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Flow>)))
  "Returns string type for a message object of type '<Flow>"
  "cf_msgs/Flow")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Flow)))
  "Returns string type for a message object of type 'Flow"
  "cf_msgs/Flow")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Flow>)))
  "Returns md5sum for a message object of type '<Flow>"
  "1355de43fdad329d668323cf3db6c71c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Flow)))
  "Returns md5sum for a message object of type 'Flow"
  "1355de43fdad329d668323cf3db6c71c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Flow>)))
  "Returns full string definition for message of type '<Flow>"
  (cl:format cl:nil "std_msgs/Header header~%int32 deltaX~%int32 deltaY~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Flow)))
  "Returns full string definition for message of type 'Flow"
  (cl:format cl:nil "std_msgs/Header header~%int32 deltaX~%int32 deltaY~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Flow>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Flow>))
  "Converts a ROS message object to a list"
  (cl:list 'Flow
    (cl:cons ':header (header msg))
    (cl:cons ':deltaX (deltaX msg))
    (cl:cons ':deltaY (deltaY msg))
))
