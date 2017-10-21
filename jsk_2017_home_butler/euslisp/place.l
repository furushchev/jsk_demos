#!/usr/bin/env roseus
;; place.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_2017_home_butler")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun place-action-cb (server goal)
  (let ((res (send server :result)))
    (send server :set-succeeded res)))

(defun main ()
  (ros::roseus "place_server")

  (let ((server (instance ros::simple-action-server :init
                          "/place"
                          jsk_2017_home_butler::PlaceAction
                          :execute-cb #'place-action-cb)))
    (ros::rate 10)
  (while (ros::ok)
    (send server :worker)
    (ros::spin-once)
    (ros::sleep))))
