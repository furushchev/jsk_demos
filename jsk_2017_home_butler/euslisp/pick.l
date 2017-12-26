#!/usr/bin/env roseus
;; pick.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_2017_home_butler")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(defparameter *detection-topic* "/kinect_head/rgb/ObjectDetection")


(defun pick-action-cb (server goal)
  (let ((obj (send goal :goal :object))
        (res (send server :result)))

    ;; TODO: debug
    (setq *type* (or obj "georgia"))
    (open-fridge-door :use-arm :larm)
    (unix:sleep 5)
    (grasp-can :use-arm :larm)
    (close-fridge :use-arm :larm)

    (send server :set-succeeded res)))

;; main
(ros::roseus "pick_server")

(pr2-init)

(setq *server*
      (instance ros::simple-action-server :init
                "/pick"
                jsk_2017_home_butler::PickAction
                :execute-cb 'pick-action-cb))
(ros::rate 10)
(while
 (ros::ok)
 (send *server* :worker)
 (ros::spin-once)
 (ros::sleep))

