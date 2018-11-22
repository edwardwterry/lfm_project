;; Auto-generated. Do not edit!


(when (boundp 'calib::Action)
  (if (not (find-package "CALIB"))
    (make-package "CALIB"))
  (shadow 'Action (find-package "CALIB")))
(unless (find-package "CALIB::ACTION")
  (make-package "CALIB::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html


(defclass calib::Action
  :super ros::object
  :slots (_target_tag _dist _angle ))

(defmethod calib::Action
  (:init
   (&key
    ((:target_tag __target_tag) 0)
    ((:dist __dist) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _target_tag (round __target_tag))
   (setq _dist (float __dist))
   (setq _angle (float __angle))
   self)
  (:target_tag
   (&optional __target_tag)
   (if __target_tag (setq _target_tag __target_tag)) _target_tag)
  (:dist
   (&optional __dist)
   (if __dist (setq _dist __dist)) _dist)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; int32 _target_tag
    4
    ;; float32 _dist
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _target_tag
       (write-long _target_tag s)
     ;; float32 _dist
       (sys::poke _dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _target_tag
     (setq _target_tag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _dist
     (setq _dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get calib::Action :md5sum-) "7529da8ba0ed13c6f3938479d680da6a")
(setf (get calib::Action :datatype-) "calib/Action")
(setf (get calib::Action :definition-)
      "int32 target_tag
float32 dist
float32 angle

")



(provide :calib/Action "7529da8ba0ed13c6f3938479d680da6a")


