;; Auto-generated. Do not edit!


(when (boundp 'cf_msgs::Tof)
  (if (not (find-package "CF_MSGS"))
    (make-package "CF_MSGS"))
  (shadow 'Tof (find-package "CF_MSGS")))
(unless (find-package "CF_MSGS::TOF")
  (make-package "CF_MSGS::TOF"))

(in-package "ROS")
;;//! \htmlinclude Tof.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cf_msgs::Tof
  :super ros::object
  :slots (_header _zrange ))

(defmethod cf_msgs::Tof
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:zrange __zrange) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _zrange (float __zrange))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:zrange
   (&optional __zrange)
   (if __zrange (setq _zrange __zrange)) _zrange)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _zrange
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _zrange
       (sys::poke _zrange (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _zrange
     (setq _zrange (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cf_msgs::Tof :md5sum-) "40b33d111238946306a2dc30865b4f80")
(setf (get cf_msgs::Tof :datatype-) "cf_msgs/Tof")
(setf (get cf_msgs::Tof :definition-)
      "std_msgs/Header header
float32 zrange
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



(provide :cf_msgs/Tof "40b33d111238946306a2dc30865b4f80")


