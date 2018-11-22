; Auto-generated. Do not edit!


(cl:in-package calib-msg)


;//! \htmlinclude Links.msg.html

(cl:defclass <Links> (roslisp-msg-protocol:ros-message)
  ((link_id
    :reader link_id
    :initarg :link_id
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (link_active_prob
    :reader link_active_prob
    :initarg :link_active_prob
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Links (<Links>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Links>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Links)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name calib-msg:<Links> is deprecated: use calib-msg:Links instead.")))

(cl:ensure-generic-function 'link_id-val :lambda-list '(m))
(cl:defmethod link_id-val ((m <Links>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calib-msg:link_id-val is deprecated.  Use calib-msg:link_id instead.")
  (link_id m))

(cl:ensure-generic-function 'link_active_prob-val :lambda-list '(m))
(cl:defmethod link_active_prob-val ((m <Links>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calib-msg:link_active_prob-val is deprecated.  Use calib-msg:link_active_prob instead.")
  (link_active_prob m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Links>) ostream)
  "Serializes a message object of type '<Links>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'link_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'link_id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'link_active_prob))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'link_active_prob))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Links>) istream)
  "Deserializes a message object of type '<Links>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'link_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'link_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'link_active_prob) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'link_active_prob)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Links>)))
  "Returns string type for a message object of type '<Links>"
  "calib/Links")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Links)))
  "Returns string type for a message object of type 'Links"
  "calib/Links")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Links>)))
  "Returns md5sum for a message object of type '<Links>"
  "20b3840007e10c6883a36f0694bd3f6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Links)))
  "Returns md5sum for a message object of type 'Links"
  "20b3840007e10c6883a36f0694bd3f6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Links>)))
  "Returns full string definition for message of type '<Links>"
  (cl:format cl:nil "int32[] link_id~%float32[] link_active_prob~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Links)))
  "Returns full string definition for message of type 'Links"
  (cl:format cl:nil "int32[] link_id~%float32[] link_active_prob~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Links>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'link_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'link_active_prob) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Links>))
  "Converts a ROS message object to a list"
  (cl:list 'Links
    (cl:cons ':link_id (link_id msg))
    (cl:cons ':link_active_prob (link_active_prob msg))
))
