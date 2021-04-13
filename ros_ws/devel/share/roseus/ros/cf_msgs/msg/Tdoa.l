;; Auto-generated. Do not edit!


(when (boundp 'cf_msgs::Tdoa)
  (if (not (find-package "CF_MSGS"))
    (make-package "CF_MSGS"))
  (shadow 'Tdoa (find-package "CF_MSGS")))
(unless (find-package "CF_MSGS::TDOA")
  (make-package "CF_MSGS::TDOA"))

(in-package "ROS")
;;//! \htmlinclude Tdoa.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cf_msgs::Tdoa
  :super ros::object
  :slots (_header _idA _idB _data ))

(defmethod cf_msgs::Tdoa
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:idA __idA) 0)
    ((:idB __idB) 0)
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _idA (round __idA))
   (setq _idB (round __idB))
   (setq _data (float __data))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:idA
   (&optional __idA)
   (if __idA (setq _idA __idA)) _idA)
  (:idB
   (&optional __idB)
   (if __idB (setq _idB __idB)) _idB)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _idA
    4
    ;; int32 _idB
    4
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _idA
       (write-long _idA s)
     ;; int32 _idB
       (write-long _idB s)
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _idA
     (setq _idA (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _idB
     (setq _idB (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cf_msgs::Tdoa :md5sum-) "b3caf99b788d7132e40204b4bdeef95d")
(setf (get cf_msgs::Tdoa :datatype-) "cf_msgs/Tdoa")
(setf (get cf_msgs::Tdoa :definition-)
      "std_msgs/Header header
int32 idA
int32 idB
float32 data
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



(provide :cf_msgs/Tdoa "b3caf99b788d7132e40204b4bdeef95d")


