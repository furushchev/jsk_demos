;; action-look-at-sound.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :action-disable "package://interactive_behavior_201409/euslisp/action-disable.l")

(ros::load-ros-manifest "jsk_hark_msgs")

(defparameter *sound-look-at-pos* nil)

(defun sound-direction->look-at-pos (msg)
  (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                   "/head_pan_link"
                       (send msg :header :stamp))))
    (if cds
        (let ((local-pos (scale 3000 (float-vector (send msg :vector :x)
                                                   (send msg :vector :y)
                                                   (send msg :vector :z)))))
          (let ((world-pos (send cds :transform-vector local-pos)))
            world-pos))
          (progn
            (ros::ros-warn "Failed to transform /base_footprint to ~A" (send msg :header :frame_id))
            nil))))

(defun sound-direction-cb (msg)
  (setq *sound-look-at-pos* (sound-direction->look-at-pos msg)))

(subscribe
 "/sound_direction"
 geometry_msgs::Vector3Stamped
 #'sound-direction-cb)

(provide :action-look-at-sound) ;; end of action-look-at-sound.l
