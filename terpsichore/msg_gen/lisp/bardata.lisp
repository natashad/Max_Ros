; Auto-generated. Do not edit!


(cl:in-package terpsichore-msg)


;//! \htmlinclude bardata.msg.html

(cl:defclass <bardata> (roslisp-msg-protocol:ros-message)
  ((beats
    :reader beats
    :initarg :beats
    :type (cl:vector terpsichore-msg:pair)
   :initform (cl:make-array 0 :element-type 'terpsichore-msg:pair :initial-element (cl:make-instance 'terpsichore-msg:pair)))
   (events
    :reader events
    :initarg :events
    :type (cl:vector terpsichore-msg:pair)
   :initform (cl:make-array 0 :element-type 'terpsichore-msg:pair :initial-element (cl:make-instance 'terpsichore-msg:pair))))
)

(cl:defclass bardata (<bardata>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bardata>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bardata)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name terpsichore-msg:<bardata> is deprecated: use terpsichore-msg:bardata instead.")))

(cl:ensure-generic-function 'beats-val :lambda-list '(m))
(cl:defmethod beats-val ((m <bardata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader terpsichore-msg:beats-val is deprecated.  Use terpsichore-msg:beats instead.")
  (beats m))

(cl:ensure-generic-function 'events-val :lambda-list '(m))
(cl:defmethod events-val ((m <bardata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader terpsichore-msg:events-val is deprecated.  Use terpsichore-msg:events instead.")
  (events m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bardata>) ostream)
  "Serializes a message object of type '<bardata>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'beats))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'beats))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'events))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'events))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bardata>) istream)
  "Deserializes a message object of type '<bardata>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'beats) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'beats)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'terpsichore-msg:pair))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'events) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'events)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'terpsichore-msg:pair))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bardata>)))
  "Returns string type for a message object of type '<bardata>"
  "terpsichore/bardata")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bardata)))
  "Returns string type for a message object of type 'bardata"
  "terpsichore/bardata")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bardata>)))
  "Returns md5sum for a message object of type '<bardata>"
  "9a317e4c8ded3becddcd48b6b516684b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bardata)))
  "Returns md5sum for a message object of type 'bardata"
  "9a317e4c8ded3becddcd48b6b516684b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bardata>)))
  "Returns full string definition for message of type '<bardata>"
  (cl:format cl:nil "pair[] beats~%pair[] events~%================================================================================~%MSG: terpsichore/pair~%float64 t~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bardata)))
  "Returns full string definition for message of type 'bardata"
  (cl:format cl:nil "pair[] beats~%pair[] events~%================================================================================~%MSG: terpsichore/pair~%float64 t~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bardata>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'beats) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'events) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bardata>))
  "Converts a ROS message object to a list"
  (cl:list 'bardata
    (cl:cons ':beats (beats msg))
    (cl:cons ':events (events msg))
))
