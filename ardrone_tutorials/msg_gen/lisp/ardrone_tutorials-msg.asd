
(cl:in-package :asdf)

(defsystem "ardrone_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StateData" :depends-on ("_package_StateData"))
    (:file "_package_StateData" :depends-on ("_package"))
  ))