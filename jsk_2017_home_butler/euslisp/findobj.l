#!/usr/bin/env roseus
;; findobj.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(ros::load-ros-manifest "jsk_2017_home_butler")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun find-object-action-cb (server goal)
  (let ((res (send server :result)))
    (ros::ros-info "find-object")
    (send server :set-succeeded res)))

(defun main ()
  (ros::roseus "findobj_server")

  (let ((server (instance ros::simple-action-server :init
                          "/find_object"
                          jsk_2017_home_butler::FindObjectAction
                          :execute-cb #'find-object-action-cb)))
    (ros::rate 10)
  (while (ros::ok)
    (send server :worker)
    (ros::spin-once)
    (ros::sleep))))

