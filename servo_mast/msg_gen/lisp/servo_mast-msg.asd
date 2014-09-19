
(cl:in-package :asdf)

(defsystem "servo_mast-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "mast_turn" :depends-on ("_package_mast_turn"))
    (:file "_package_mast_turn" :depends-on ("_package"))
    (:file "mast_position" :depends-on ("_package_mast_position"))
    (:file "_package_mast_position" :depends-on ("_package"))
  ))