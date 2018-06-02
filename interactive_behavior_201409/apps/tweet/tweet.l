#!/usr/bin/env roseus
;; tweet.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")

;; hack
(when (not (ros::has-param "/active_user/elapsed_time"))
  (ros::set-param "/active_user/elapsed_time" 0))

(require :tweet-client "package://jsk_robot_startup/lifelog/tweet_client.l")
(require :speak "package://pr2eus/speak.l")

(defparameter *camera-topic* "/kinect_head/rgb/image_rect_color")

(defun main ()
  (let ((content (cdr (assoc "content" *param* :test #'string=))))
    (ros::ros-info "content: ~A" content)
    (when content
      (speak-jp "写真もとりませんか？")
      (unix:sleep 2)
      (speak-jp "さん")
      (unix:sleep 1)
      (speak-jp "に")
      (unix:sleep 1)
      (speak-jp "いち")
      (unix:sleep 1)
      (speak-jp "カシャ！")
      (tweet-string content
                    :warning-time nil
                    :with-image *camera-topic*))))

(if (main) (exit 0) (exit 1))