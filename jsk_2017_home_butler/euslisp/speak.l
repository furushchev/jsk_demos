#!/usr/bin/env roseus
;; speak.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_2017_home_butler")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun speak-action-cb (server goal)
  (let ((res (send server :result))
        (content (send goal :goal :content)))
    (send *ri* :speak-en "Hi!")
    (send *ri* :speak-en content)
    (send server :set-succeeded res)))

;; main
(ros::roseus "speak_server")

(pr2-init)

(setq *server*
      (instance ros::simple-action-server :init
                "/speak"
                jsk_2017_home_butler::SpeakAction
                :execute-cb 'speak-action-cb))

(ros::rate 10)
(while
 (ros::ok)
 (send *server* :worker)
 (ros::spin-once)
 (ros::sleep))
