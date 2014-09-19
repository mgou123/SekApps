; Auto-generated. Do not edit!


(cl:in-package servo_mast-msg)


;//! \htmlinclude mast_position.msg.html

(cl:defclass <mast_position> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0))
)

(cl:defclass mast_position (<mast_position>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mast_position>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mast_position)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name servo_mast-msg:<mast_position> is deprecated: use servo_mast-msg:mast_position instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <mast_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_mast-msg:header-val is deprecated.  Use servo_mast-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <mast_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_mast-msg:position-val is deprecated.  Use servo_mast-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mast_position>) ostream)
  "Serializes a message object of type '<mast_position>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mast_position>) istream)
  "Deserializes a message object of type '<mast_position>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mast_position>)))
  "Returns string type for a message object of type '<mast_position>"
  "servo_mast/mast_position")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mast_position)))
  "Returns string type for a message object of type 'mast_position"
  "servo_mast/mast_position")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mast_position>)))
  "Returns md5sum for a message object of type '<mast_position>"
  "41da98170d2fba45d8d2070252f31142")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mast_position)))
  "Returns md5sum for a message object of type 'mast_position"
  "41da98170d2fba45d8d2070252f31142")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mast_position>)))
  "Returns full string definition for message of type '<mast_position>"
  (cl:format cl:nil "Header header~%float64 position~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mast_position)))
  "Returns full string definition for message of type 'mast_position"
  (cl:format cl:nil "Header header~%float64 position~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mast_position>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mast_position>))
  "Converts a ROS message object to a list"
  (cl:list 'mast_position
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
))
