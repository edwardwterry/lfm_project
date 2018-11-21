;; Auto-generated. Do not edit!


(when (boundp 'lfm::Block)
  (if (not (find-package "LFM"))
    (make-package "LFM"))
  (shadow 'Block (find-package "LFM")))
(unless (find-package "LFM::BLOCK")
  (make-package "LFM::BLOCK"))

(in-package "ROS")
;;//! \htmlinclude Block.msg.html


(defclass lfm::Block
  :super ros::object
  :slots (_block_id _block_x _block_y ))

(defmethod lfm::Block
  (:init
   (&key
    ((:block_id __block_id) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:block_x __block_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:block_y __block_y) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _block_id __block_id)
   (setq _block_x __block_x)
   (setq _block_y __block_y)
   self)
  (:block_id
   (&optional __block_id)
   (if __block_id (setq _block_id __block_id)) _block_id)
  (:block_x
   (&optional __block_x)
   (if __block_x (setq _block_x __block_x)) _block_x)
  (:block_y
   (&optional __block_y)
   (if __block_y (setq _block_y __block_y)) _block_y)
  (:serialization-length
   ()
   (+
    ;; float32[] _block_id
    (* 4    (length _block_id)) 4
    ;; float32[] _block_x
    (* 4    (length _block_x)) 4
    ;; float32[] _block_y
    (* 4    (length _block_y)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _block_id
     (write-long (length _block_id) s)
     (dotimes (i (length _block_id))
       (sys::poke (elt _block_id i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _block_x
     (write-long (length _block_x) s)
     (dotimes (i (length _block_x))
       (sys::poke (elt _block_x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _block_y
     (write-long (length _block_y) s)
     (dotimes (i (length _block_y))
       (sys::poke (elt _block_y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _block_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _block_id (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _block_id i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _block_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _block_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _block_x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _block_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _block_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _block_y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get lfm::Block :md5sum-) "9643fae418351e15e5ed3fde4997c5fa")
(setf (get lfm::Block :datatype-) "lfm/Block")
(setf (get lfm::Block :definition-)
      "float32[] block_id
float32[] block_x
float32[] block_y

")



(provide :lfm/Block "9643fae418351e15e5ed3fde4997c5fa")


