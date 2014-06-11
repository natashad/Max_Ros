; Auto-generated. Do not edit!


(cl:in-package terpsichore-msg)


;//! \htmlinclude pair.msg.html

(cl:defclass <pair> (roslisp-msg-protocol:ros-message)
  ((t
    :reader t
    :initarg :t
    :type cl:float
    :initform 0.0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass pair (<pair>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pair>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pair)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name terpsichore-msg:<pair> is deprecated: use terpsichore-msg:pair instead.")))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <pair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader terpsichore-msg:t-val is deprecated.  Use terpsichore-msg:t instead.")
  (t m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <pair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader terpsichore-msg:data-val is deprecated.  Use terpsichore-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pair>) ostream)
  "Serializes a message object of type '<pair>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 't))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pair>) istream)
  "Deserializes a message object of type '<pair>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pair>)))
  "Returns string type for a message object of type '<pair>"
  "terpsichore/pair")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pair)))
  "Returns string type for a message object of type 'pair"
  "terpsichore/pair")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pair>)))
  "Returns md5sum for a message object of type '<pair>"
  "4704d1e9bdec53dad180dbd02389b120")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pair)))
  "Returns md5sum for a message object of type 'pair"
  "4704d1e9bdec53dad180dbd02389b120")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pair>)))
  "Returns full string definition for message of type '<pair>"
  (cl:format cl:nil "float64 t~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pair)))
  "Returns full string definition for message of type 'pair"
  (cl:format cl:nil "float64 t~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pair>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pair>))
  "Converts a ROS message object to a list"
  (cl:list 'pair
    (cl:cons ':t (t msg))
    (cl:cons ':data (data msg))
))
