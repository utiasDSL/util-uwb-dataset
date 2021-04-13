;; Auto-generated. Do not edit!


(when (boundp 'cf_msgs::Baro)
  (if (not (find-package "CF_MSGS"))
    (make-package "CF_MSGS"))
  (shadow 'Baro (find-package "CF_MSGS")))
(unless (find-package "CF_MSGS::BARO")
  (make-package "CF_MSGS::BARO"))

(in-package "ROS")
;;//! \htmlinclude Baro.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cf_msgs::Baro
  :super ros::object
  :slots (_header _asl ))

(defmethod cf_msgs::Baro
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:asl __asl) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _asl (float __asl))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:asl
   (&optional __asl)
   (if __asl (setq _asl __asl)) _asl)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _asl
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _asl
       (sys::poke _asl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _asl
     (setq _asl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cf_msgs::Baro :md5sum-) "ab78116f347228ddb2bd7524f612cb42")
(setf (get cf_msgs::Baro :datatype-) "cf_msgs/Baro")
(setf (get cf_msgs::Baro :definition-)
      "std_msgs/Header header
float32 asl
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :cf_msgs/Baro "ab78116f347228ddb2bd7524f612cb42")


