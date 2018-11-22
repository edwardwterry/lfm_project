;; Auto-generated. Do not edit!


(when (boundp 'calib::Links)
  (if (not (find-package "CALIB"))
    (make-package "CALIB"))
  (shadow 'Links (find-package "CALIB")))
(unless (find-package "CALIB::LINKS")
  (make-package "CALIB::LINKS"))

(in-package "ROS")
;;//! \htmlinclude Links.msg.html


(defclass calib::Links
  :super ros::object
  :slots (_link_id _link_active_prob ))

(defmethod calib::Links
  (:init
   (&key
    ((:link_id __link_id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:link_active_prob __link_active_prob) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _link_id __link_id)
   (setq _link_active_prob __link_active_prob)
   self)
  (:link_id
   (&optional __link_id)
   (if __link_id (setq _link_id __link_id)) _link_id)
  (:link_active_prob
   (&optional __link_active_prob)
   (if __link_active_prob (setq _link_active_prob __link_active_prob)) _link_active_prob)
  (:serialization-length
   ()
   (+
    ;; int32[] _link_id
    (* 4    (length _link_id)) 4
    ;; float32[] _link_active_prob
    (* 4    (length _link_active_prob)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _link_id
     (write-long (length _link_id) s)
     (dotimes (i (length _link_id))
       (write-long (elt _link_id i) s)
       )
     ;; float32[] _link_active_prob
     (write-long (length _link_active_prob) s)
     (dotimes (i (length _link_active_prob))
       (sys::poke (elt _link_active_prob i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _link_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _link_id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _link_id i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _link_active_prob
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _link_active_prob (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _link_active_prob i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get calib::Links :md5sum-) "20b3840007e10c6883a36f0694bd3f6b")
(setf (get calib::Links :datatype-) "calib/Links")
(setf (get calib::Links :definition-)
      "int32[] link_id
float32[] link_active_prob

")



(provide :calib/Links "20b3840007e10c6883a36f0694bd3f6b")


