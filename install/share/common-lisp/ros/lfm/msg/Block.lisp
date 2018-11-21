; Auto-generated. Do not edit!


(cl:in-package lfm-msg)


;//! \htmlinclude Block.msg.html

(cl:defclass <Block> (roslisp-msg-protocol:ros-message)
  ((block_id
    :reader block_id
    :initarg :block_id
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (block_x
    :reader block_x
    :initarg :block_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (block_y
    :reader block_y
    :initarg :block_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Block (<Block>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Block>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Block)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lfm-msg:<Block> is deprecated: use lfm-msg:Block instead.")))

(cl:ensure-generic-function 'block_id-val :lambda-list '(m))
(cl:defmethod block_id-val ((m <Block>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lfm-msg:block_id-val is deprecated.  Use lfm-msg:block_id instead.")
  (block_id m))

(cl:ensure-generic-function 'block_x-val :lambda-list '(m))
(cl:defmethod block_x-val ((m <Block>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lfm-msg:block_x-val is deprecated.  Use lfm-msg:block_x instead.")
  (block_x m))

(cl:ensure-generic-function 'block_y-val :lambda-list '(m))
(cl:defmethod block_y-val ((m <Block>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lfm-msg:block_y-val is deprecated.  Use lfm-msg:block_y instead.")
  (block_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Block>) ostream)
  "Serializes a message object of type '<Block>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'block_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'block_id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'block_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'block_x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'block_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'block_y))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Block>) istream)
  "Deserializes a message object of type '<Block>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'block_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'block_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'block_x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'block_x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'block_y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'block_y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Block>)))
  "Returns string type for a message object of type '<Block>"
  "lfm/Block")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Block)))
  "Returns string type for a message object of type 'Block"
  "lfm/Block")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Block>)))
  "Returns md5sum for a message object of type '<Block>"
  "9643fae418351e15e5ed3fde4997c5fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Block)))
  "Returns md5sum for a message object of type 'Block"
  "9643fae418351e15e5ed3fde4997c5fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Block>)))
  "Returns full string definition for message of type '<Block>"
  (cl:format cl:nil "float32[] block_id~%float32[] block_x~%float32[] block_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Block)))
  "Returns full string definition for message of type 'Block"
  (cl:format cl:nil "float32[] block_id~%float32[] block_x~%float32[] block_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Block>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'block_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'block_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'block_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Block>))
  "Converts a ROS message object to a list"
  (cl:list 'Block
    (cl:cons ':block_id (block_id msg))
    (cl:cons ':block_x (block_x msg))
    (cl:cons ':block_y (block_y msg))
))
