; Auto-generated. Do not edit!


(cl:in-package cf_msgs-msg)


;//! \htmlinclude Tof.msg.html

(cl:defclass <Tof> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (zrange
    :reader zrange
    :initarg :zrange
    :type cl:float
    :initform 0.0))
)

(cl:defclass Tof (<Tof>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tof>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tof)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cf_msgs-msg:<Tof> is deprecated: use cf_msgs-msg:Tof instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Tof>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:header-val is deprecated.  Use cf_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'zrange-val :lambda-list '(m))
(cl:defmethod zrange-val ((m <Tof>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:zrange-val is deprecated.  Use cf_msgs-msg:zrange instead.")
  (zrange m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tof>) ostream)
  "Serializes a message object of type '<Tof>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zrange))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tof>) istream)
  "Deserializes a message object of type '<Tof>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zrange) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tof>)))
  "Returns string type for a message object of type '<Tof>"
  "cf_msgs/Tof")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tof)))
  "Returns string type for a message object of type 'Tof"
  "cf_msgs/Tof")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tof>)))
  "Returns md5sum for a message object of type '<Tof>"
  "40b33d111238946306a2dc30865b4f80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tof)))
  "Returns md5sum for a message object of type 'Tof"
  "40b33d111238946306a2dc30865b4f80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tof>)))
  "Returns full string definition for message of type '<Tof>"
  (cl:format cl:nil "std_msgs/Header header~%float32 zrange~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tof)))
  "Returns full string definition for message of type 'Tof"
  (cl:format cl:nil "std_msgs/Header header~%float32 zrange~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tof>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tof>))
  "Converts a ROS message object to a list"
  (cl:list 'Tof
    (cl:cons ':header (header msg))
    (cl:cons ':zrange (zrange msg))
))
