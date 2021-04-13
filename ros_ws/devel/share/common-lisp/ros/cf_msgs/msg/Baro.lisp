; Auto-generated. Do not edit!


(cl:in-package cf_msgs-msg)


;//! \htmlinclude Baro.msg.html

(cl:defclass <Baro> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (asl
    :reader asl
    :initarg :asl
    :type cl:float
    :initform 0.0))
)

(cl:defclass Baro (<Baro>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Baro>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Baro)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cf_msgs-msg:<Baro> is deprecated: use cf_msgs-msg:Baro instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Baro>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:header-val is deprecated.  Use cf_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'asl-val :lambda-list '(m))
(cl:defmethod asl-val ((m <Baro>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:asl-val is deprecated.  Use cf_msgs-msg:asl instead.")
  (asl m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Baro>) ostream)
  "Serializes a message object of type '<Baro>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'asl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Baro>) istream)
  "Deserializes a message object of type '<Baro>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'asl) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Baro>)))
  "Returns string type for a message object of type '<Baro>"
  "cf_msgs/Baro")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Baro)))
  "Returns string type for a message object of type 'Baro"
  "cf_msgs/Baro")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Baro>)))
  "Returns md5sum for a message object of type '<Baro>"
  "ab78116f347228ddb2bd7524f612cb42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Baro)))
  "Returns md5sum for a message object of type 'Baro"
  "ab78116f347228ddb2bd7524f612cb42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Baro>)))
  "Returns full string definition for message of type '<Baro>"
  (cl:format cl:nil "std_msgs/Header header~%float32 asl~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Baro)))
  "Returns full string definition for message of type 'Baro"
  (cl:format cl:nil "std_msgs/Header header~%float32 asl~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Baro>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Baro>))
  "Converts a ROS message object to a list"
  (cl:list 'Baro
    (cl:cons ':header (header msg))
    (cl:cons ':asl (asl msg))
))
