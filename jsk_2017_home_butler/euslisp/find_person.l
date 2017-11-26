#!/usr/bin/env roseus
;; findppl.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(ros::load-ros-manifest "jsk_2017_home_butler")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun find-person-action-cb (server goal)
  (let ((res (send server :result)))
    (send server :set-succeeded res)))

;; main
(ros::roseus "find_person_server")

(setq *server*
      (instance ros::simple-action-server :init
                "/find_person"
                jsk_2017_home_butler::FindPersonAction
                :execute-cb 'find-person-action-cb))

(ros::rate 10)
(while
 (ros::ok)
 (send *server* :worker)
 (ros::spin-once)
 (ros::sleep))
