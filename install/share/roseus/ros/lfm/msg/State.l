;; Auto-generated. Do not edit!


(when (boundp 'lfm::State)
  (if (not (find-package "LFM"))
    (make-package "LFM"))
  (shadow 'State (find-package "LFM")))
(unless (find-package "LFM::STATE")
  (make-package "LFM::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lfm::State
  :super ros::object
  :slots (_header _state ))

(defmethod lfm::State
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) (instance lfm::Block :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state __state)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&rest __state)
   (if (keywordp (car __state))
       (send* _state __state)
     (progn
       (if __state (setq _state (car __state)))
       _state)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; lfm/Block _state
    (send _state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; lfm/Block _state
       (send _state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; lfm/Block _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;;
   self)
  )

(setf (get lfm::State :md5sum-) "1aa313c94c0cf265aab8a999d387041e")
(setf (get lfm::State :datatype-) "lfm/State")
(setf (get lfm::State :definition-)
      "std_msgs/Header header
Block state

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: lfm/Block
float32[] block_id
float32[] block_x
float32[] block_y

")



(provide :lfm/State "1aa313c94c0cf265aab8a999d387041e")


