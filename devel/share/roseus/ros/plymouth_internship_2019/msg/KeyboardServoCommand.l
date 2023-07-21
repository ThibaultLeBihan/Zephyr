;; Auto-generated. Do not edit!


(when (boundp 'plymouth_internship_2019::KeyboardServoCommand)
  (if (not (find-package "PLYMOUTH_INTERNSHIP_2019"))
    (make-package "PLYMOUTH_INTERNSHIP_2019"))
  (shadow 'KeyboardServoCommand (find-package "PLYMOUTH_INTERNSHIP_2019")))
(unless (find-package "PLYMOUTH_INTERNSHIP_2019::KEYBOARDSERVOCOMMAND")
  (make-package "PLYMOUTH_INTERNSHIP_2019::KEYBOARDSERVOCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude KeyboardServoCommand.msg.html


(defclass plymouth_internship_2019::KeyboardServoCommand
  :super ros::object
  :slots (_servo_command_1 _servo_command_2 ))

(defmethod plymouth_internship_2019::KeyboardServoCommand
  (:init
   (&key
    ((:servo_command_1 __servo_command_1) 0)
    ((:servo_command_2 __servo_command_2) 0)
    )
   (send-super :init)
   (setq _servo_command_1 (round __servo_command_1))
   (setq _servo_command_2 (round __servo_command_2))
   self)
  (:servo_command_1
   (&optional __servo_command_1)
   (if __servo_command_1 (setq _servo_command_1 __servo_command_1)) _servo_command_1)
  (:servo_command_2
   (&optional __servo_command_2)
   (if __servo_command_2 (setq _servo_command_2 __servo_command_2)) _servo_command_2)
  (:serialization-length
   ()
   (+
    ;; int64 _servo_command_1
    8
    ;; int64 _servo_command_2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _servo_command_1
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _servo_command_1 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _servo_command_1) (= (length (_servo_command_1 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_servo_command_1 . bv) 0) 0) s)
              (write-long (ash (elt (_servo_command_1 . bv) 1) -1) s))
             ((and (class _servo_command_1) (= (length (_servo_command_1 . bv)) 1)) ;; big1
              (write-long (elt (_servo_command_1 . bv) 0) s)
              (write-long (if (>= _servo_command_1 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _servo_command_1 s)(write-long (if (>= _servo_command_1 0) 0 #xffffffff) s)))
     ;; int64 _servo_command_2
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _servo_command_2 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _servo_command_2) (= (length (_servo_command_2 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_servo_command_2 . bv) 0) 0) s)
              (write-long (ash (elt (_servo_command_2 . bv) 1) -1) s))
             ((and (class _servo_command_2) (= (length (_servo_command_2 . bv)) 1)) ;; big1
              (write-long (elt (_servo_command_2 . bv) 0) s)
              (write-long (if (>= _servo_command_2 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _servo_command_2 s)(write-long (if (>= _servo_command_2 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _servo_command_1
#+(or :alpha :irix6 :x86_64)
      (setf _servo_command_1 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _servo_command_1 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _servo_command_2
#+(or :alpha :irix6 :x86_64)
      (setf _servo_command_2 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _servo_command_2 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get plymouth_internship_2019::KeyboardServoCommand :md5sum-) "9dbce863de6b75635fc5625ec0af5414")
(setf (get plymouth_internship_2019::KeyboardServoCommand :datatype-) "plymouth_internship_2019/KeyboardServoCommand")
(setf (get plymouth_internship_2019::KeyboardServoCommand :definition-)
      "int64 servo_command_1
int64 servo_command_2

")



(provide :plymouth_internship_2019/KeyboardServoCommand "9dbce863de6b75635fc5625ec0af5414")


