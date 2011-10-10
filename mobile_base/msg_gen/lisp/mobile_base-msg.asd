
(cl:in-package :asdf)

(defsystem "mobile_base-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "tweak" :depends-on ("_package_tweak"))
    (:file "_package_tweak" :depends-on ("_package"))
  ))