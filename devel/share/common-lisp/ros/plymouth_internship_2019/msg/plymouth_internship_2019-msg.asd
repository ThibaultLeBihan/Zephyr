
(cl:in-package :asdf)

(defsystem "plymouth_internship_2019-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "KeyboardServoCommand" :depends-on ("_package_KeyboardServoCommand"))
    (:file "_package_KeyboardServoCommand" :depends-on ("_package"))
  ))