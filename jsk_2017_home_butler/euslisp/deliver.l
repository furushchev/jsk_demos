#!/usr/bin/env roseus
;; deliver.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(ros::load-ros-manifest "jsk_2017_home_butler")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")

(defun deliver-action-cb (server goal)
  (let ((res (send server :result)))

    ;; TODO: debug
    (hand-over :larm :wait-shock t)

    (send server :set-succeeded res)))

;; main
(ros::roseus "deliver_server")

(setq *server*
      (instance ros::simple-action-server :init
                "/deliver"
                jsk_2017_home_butler::DeliverAction
                :execute-cb 'deliver-action-cb))
(ros::rate 10)
(while
 (ros::ok)
 (send *server* :worker)
 (ros::spin-once)
 (ros::sleep))
