; Auto-generated. Do not edit!


(cl:in-package lfm-msg)


;//! \htmlinclude Action.msg.html

(cl:defclass <Action> (roslisp-msg-protocol:ros-message)
  ((target_tag
    :reader target_tag
    :initarg :target_tag
    :type cl:integer
    :initform 0)
   (dist
    :reader dist
    :initarg :dist
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Action (<Action>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Action>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Action)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lfm-msg:<Action> is deprecated: use lfm-msg:Action instead.")))

(cl:ensure-generic-function 'target_tag-val :lambda-list '(m))
(cl:defmethod target_tag-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lfm-msg:target_tag-val is deprecated.  Use lfm-msg:target_tag instead.")
  (target_tag m))

(cl:ensure-generic-function 'dist-val :lambda-list '(m))
(cl:defmethod dist-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lfm-msg:dist-val is deprecated.  Use lfm-msg:dist instead.")
  (dist m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lfm-msg:angle-val is deprecated.  Use lfm-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Action>) ostream)
  "Serializes a message object of type '<Action>"
  (cl:let* ((signed (cl:slot-value msg 'target_tag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Action>) istream)
  "Deserializes a message object of type '<Action>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_tag) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Action>)))
  "Returns string type for a message object of type '<Action>"
  "lfm/Action")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Action)))
  "Returns string type for a message object of type 'Action"
  "lfm/Action")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Action>)))
  "Returns md5sum for a message object of type '<Action>"
  "7529da8ba0ed13c6f3938479d680da6a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Action)))
  "Returns md5sum for a message object of type 'Action"
  "7529da8ba0ed13c6f3938479d680da6a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Action>)))
  "Returns full string definition for message of type '<Action>"
  (cl:format cl:nil "int32 target_tag~%float32 dist~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Action)))
  "Returns full string definition for message of type 'Action"
  (cl:format cl:nil "int32 target_tag~%float32 dist~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Action>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Action>))
  "Converts a ROS message object to a list"
  (cl:list 'Action
    (cl:cons ':target_tag (target_tag msg))
    (cl:cons ':dist (dist msg))
    (cl:cons ':angle (angle msg))
))
