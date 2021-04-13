; Auto-generated. Do not edit!


(cl:in-package cf_msgs-msg)


;//! \htmlinclude Tdoa.msg.html

(cl:defclass <Tdoa> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (idA
    :reader idA
    :initarg :idA
    :type cl:integer
    :initform 0)
   (idB
    :reader idB
    :initarg :idB
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass Tdoa (<Tdoa>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tdoa>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tdoa)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cf_msgs-msg:<Tdoa> is deprecated: use cf_msgs-msg:Tdoa instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Tdoa>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:header-val is deprecated.  Use cf_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'idA-val :lambda-list '(m))
(cl:defmethod idA-val ((m <Tdoa>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:idA-val is deprecated.  Use cf_msgs-msg:idA instead.")
  (idA m))

(cl:ensure-generic-function 'idB-val :lambda-list '(m))
(cl:defmethod idB-val ((m <Tdoa>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:idB-val is deprecated.  Use cf_msgs-msg:idB instead.")
  (idB m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Tdoa>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cf_msgs-msg:data-val is deprecated.  Use cf_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tdoa>) ostream)
  "Serializes a message object of type '<Tdoa>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'idA)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'idB)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tdoa>) istream)
  "Deserializes a message object of type '<Tdoa>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'idA) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'idB) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tdoa>)))
  "Returns string type for a message object of type '<Tdoa>"
  "cf_msgs/Tdoa")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tdoa)))
  "Returns string type for a message object of type 'Tdoa"
  "cf_msgs/Tdoa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tdoa>)))
  "Returns md5sum for a message object of type '<Tdoa>"
  "b3caf99b788d7132e40204b4bdeef95d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tdoa)))
  "Returns md5sum for a message object of type 'Tdoa"
  "b3caf99b788d7132e40204b4bdeef95d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tdoa>)))
  "Returns full string definition for message of type '<Tdoa>"
  (cl:format cl:nil "std_msgs/Header header~%int32 idA~%int32 idB~%float32 data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tdoa)))
  "Returns full string definition for message of type 'Tdoa"
  (cl:format cl:nil "std_msgs/Header header~%int32 idA~%int32 idB~%float32 data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tdoa>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tdoa>))
  "Converts a ROS message object to a list"
  (cl:list 'Tdoa
    (cl:cons ':header (header msg))
    (cl:cons ':idA (idA msg))
    (cl:cons ':idB (idB msg))
    (cl:cons ':data (data msg))
))
