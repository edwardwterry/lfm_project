;; Auto-generated. Do not edit!


(when (boundp 'lfm::Action)
  (if (not (find-package "LFM"))
    (make-package "LFM"))
  (shadow 'Action (find-package "LFM")))
(unless (find-package "LFM::ACTION")
  (make-package "LFM::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html


(defclass lfm::Action
  :super ros::object
  :slots (_target_tag _z _dist _angle ))

(defmethod lfm::Action
  (:init
   (&key
    ((:target_tag __target_tag) 0)
    ((:z __z) 0.0)
    ((:dist __dist) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _target_tag (round __target_tag))
   (setq _z (float __z))
   (setq _dist (float __dist))
   (setq _angle (float __angle))
   self)
  (:target_tag
   (&optional __target_tag)
   (if __target_tag (setq _target_tag __target_tag)) _target_tag)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
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
    ;; float32 _z
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
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dist
     (setq _dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get lfm::Action :md5sum-) "5df7593a24a770255fa2ccf8e16dcf4f")
(setf (get lfm::Action :datatype-) "lfm/Action")
(setf (get lfm::Action :definition-)
      "int32 target_tag
float32 z
float32 dist
float32 angle

")



(provide :lfm/Action "5df7593a24a770255fa2ccf8e16dcf4f")


