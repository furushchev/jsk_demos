#!/usr/bin/env roseus
;; goto.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_2017_home_butler")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(defun go-action-cb (server goal)
  (let ((res (send server :result))
        (spot (send goal :goal :location)))
    (ros::ros-info "spot: ~A" spot)
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

;; main
(ros::roseus "go_server")

(pr2-init)
(setq *scene* (make-eng2-scene))

(setq *server*
      (instance ros::simple-action-server :init
                "/go"
                jsk_2017_home_butler::GoAction
                :execute-cb #'go-action-cb)))
(ros::rate 10)
(while
 (ros::ok)
 (send *server* :worker)
 (ros::spin-once)
 (ros::sleep))
