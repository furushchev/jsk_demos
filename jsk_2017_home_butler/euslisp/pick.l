#!/usr/bin/env roseus
;; pick.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_2017_home_butler")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun pick-action-cb (server goal)
  (let ((res (send server :result)))
    (send server :set-succeeded res)))

(defun main ()
  (ros::roseus "pick_server")

  (let ((server (instance ros::simple-action-server :init
                          "/pick"
                          jsk_2017_home_butler::PickAction
                          :execute-cb #'pick-action-cb)))
    (ros::rate 10)
  (while (ros::ok)
    (send server :worker)
    (ros::spin-once)
    (ros::sleep))))
