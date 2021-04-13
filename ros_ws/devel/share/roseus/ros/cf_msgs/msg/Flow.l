;; Auto-generated. Do not edit!


(when (boundp 'cf_msgs::Flow)
  (if (not (find-package "CF_MSGS"))
    (make-package "CF_MSGS"))
  (shadow 'Flow (find-package "CF_MSGS")))
(unless (find-package "CF_MSGS::FLOW")
  (make-package "CF_MSGS::FLOW"))

(in-package "ROS")
;;//! \htmlinclude Flow.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cf_msgs::Flow
  :super ros::object
  :slots (_header _deltaX _deltaY ))

(defmethod cf_msgs::Flow
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:deltaX __deltaX) 0)
    ((:deltaY __deltaY) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _deltaX (round __deltaX))
   (setq _deltaY (round __deltaY))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:deltaX
   (&optional __deltaX)
   (if __deltaX (setq _deltaX __deltaX)) _deltaX)
  (:deltaY
   (&optional __deltaY)
   (if __deltaY (setq _deltaY __deltaY)) _deltaY)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _deltaX
    4
    ;; int32 _deltaY
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _deltaX
       (write-long _deltaX s)
     ;; int32 _deltaY
       (write-long _deltaY s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _deltaX
     (setq _deltaX (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _deltaY
     (setq _deltaY (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cf_msgs::Flow :md5sum-) "1355de43fdad329d668323cf3db6c71c")
(setf (get cf_msgs::Flow :datatype-) "cf_msgs/Flow")
(setf (get cf_msgs::Flow :definition-)
      "std_msgs/Header header
int32 deltaX
int32 deltaY
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



(provide :cf_msgs/Flow "1355de43fdad329d668323cf3db6c71c")


