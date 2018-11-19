
(cl:in-package :asdf)

(defsystem "lfm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Action" :depends-on ("_package_Action"))
    (:file "_package_Action" :depends-on ("_package"))
    (:file "AprilTagDetection" :depends-on ("_package_AprilTagDetection"))
    (:file "_package_AprilTagDetection" :depends-on ("_package"))
    (:file "AprilTagDetectionArray" :depends-on ("_package_AprilTagDetectionArray"))
    (:file "_package_AprilTagDetectionArray" :depends-on ("_package"))
    (:file "Block" :depends-on ("_package_Block"))
    (:file "_package_Block" :depends-on ("_package"))
    (:file "Links" :depends-on ("_package_Links"))
    (:file "_package_Links" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
  ))