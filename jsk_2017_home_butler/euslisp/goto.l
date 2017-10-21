#!/usr/bin/env roseus
;; goto.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_2017_home_butler")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(defun goto-action-cb (server goal)
  (let ((res (send server :result))
        (spot (send goal :goal :destination)))
    (when (or
           (null spot)
           (string= "" spot)
           (null (send *scene* :spot spot)))
      (ros::ros-error "Could not understand the spot: ~A" spot)
      (send server :set-aborted res
            (format nil "Could not understand spot: ~A" spot))
      (return-from goto-action-cb t))
    (send *ri* :move-to (send *scene* :spot spot))
    (send server :set-succeeded res)))

(defun main ()
  (ros::roseus "deliver_server")

  (pr2-init)
  (send *scene* (make-eng2-scene))

  (let ((server (instance ros::simple-action-server :init
                          "/goto"
                          jsk_2017_home_butler::GoToAction
                          :execute-cb #'goto-action-cb)))
    (ros::rate 10)
  (while (ros::ok)
    (send server :worker)
    (ros::spin-once)
    (ros::sleep))))
