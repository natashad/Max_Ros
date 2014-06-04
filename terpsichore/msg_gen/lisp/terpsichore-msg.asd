
(cl:in-package :asdf)

(defsystem "terpsichore-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pair" :depends-on ("_package_pair"))
    (:file "_package_pair" :depends-on ("_package"))
    (:file "bardata" :depends-on ("_package_bardata"))
    (:file "_package_bardata" :depends-on ("_package"))
  ))