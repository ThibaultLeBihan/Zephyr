; Auto-generated. Do not edit!


(cl:in-package plymouth_internship_2019-msg)


;//! \htmlinclude KeyboardServoCommand.msg.html

(cl:defclass <KeyboardServoCommand> (roslisp-msg-protocol:ros-message)
  ((servo_command_1
    :reader servo_command_1
    :initarg :servo_command_1
    :type cl:integer
    :initform 0)
   (servo_command_2
    :reader servo_command_2
    :initarg :servo_command_2
    :type cl:integer
    :initform 0))
)

(cl:defclass KeyboardServoCommand (<KeyboardServoCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyboardServoCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyboardServoCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plymouth_internship_2019-msg:<KeyboardServoCommand> is deprecated: use plymouth_internship_2019-msg:KeyboardServoCommand instead.")))

(cl:ensure-generic-function 'servo_command_1-val :lambda-list '(m))
(cl:defmethod servo_command_1-val ((m <KeyboardServoCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plymouth_internship_2019-msg:servo_command_1-val is deprecated.  Use plymouth_internship_2019-msg:servo_command_1 instead.")
  (servo_command_1 m))

(cl:ensure-generic-function 'servo_command_2-val :lambda-list '(m))
(cl:defmethod servo_command_2-val ((m <KeyboardServoCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plymouth_internship_2019-msg:servo_command_2-val is deprecated.  Use plymouth_internship_2019-msg:servo_command_2 instead.")
  (servo_command_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyboardServoCommand>) ostream)
  "Serializes a message object of type '<KeyboardServoCommand>"
  (cl:let* ((signed (cl:slot-value msg 'servo_command_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'servo_command_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyboardServoCommand>) istream)
  "Deserializes a message object of type '<KeyboardServoCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo_command_1) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo_command_2) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyboardServoCommand>)))
  "Returns string type for a message object of type '<KeyboardServoCommand>"
  "plymouth_internship_2019/KeyboardServoCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyboardServoCommand)))
  "Returns string type for a message object of type 'KeyboardServoCommand"
  "plymouth_internship_2019/KeyboardServoCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyboardServoCommand>)))
  "Returns md5sum for a message object of type '<KeyboardServoCommand>"
  "9dbce863de6b75635fc5625ec0af5414")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyboardServoCommand)))
  "Returns md5sum for a message object of type 'KeyboardServoCommand"
  "9dbce863de6b75635fc5625ec0af5414")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyboardServoCommand>)))
  "Returns full string definition for message of type '<KeyboardServoCommand>"
  (cl:format cl:nil "int64 servo_command_1~%int64 servo_command_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyboardServoCommand)))
  "Returns full string definition for message of type 'KeyboardServoCommand"
  (cl:format cl:nil "int64 servo_command_1~%int64 servo_command_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyboardServoCommand>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyboardServoCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyboardServoCommand
    (cl:cons ':servo_command_1 (servo_command_1 msg))
    (cl:cons ':servo_command_2 (servo_command_2 msg))
))
