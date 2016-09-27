;; action-look-at-leg.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :action-disable "package://interactive_behavior_201409/euslisp/action-disable.l")

(ros::load-ros-manifest "people_msgs")

(defparameter *people-look-at-pos* nil)

(defun people->look-at-pos (msg &optional idx)
  "return coords of people[idx] for look-at. if idx is not specified, use person that has the highest reliability"
  (let ((cds (send *tfl* :lookup-transform "/base_footprint"
                   (send msg :header :frame_id)
                   (send msg :header :stamp))))
    (if cds
        (progn
          (let* ((people (send msg :people))
                 (person (if idx (elt people idx)
                           (car (sort people #'(lambda (a b)
                                                 (> (send a :reliability) (send b :reliability)))))))
                 (world-pos (send cds :transform-vector (ros::tf-point->pos (send person :position))))
                 (distance-from-robot (distance
                                       (float-vector (elt world-pos 0)
                                                     (elt world-pos 1)
                                                     0)
                                       (float-vector 0 0 0))))
            (when (< (elt world-pos 2) 700.0)
              (setf (elt world-pos 2) 1300.0))
            (if (< distance-from-robot 2000.0)
                world-pos
              (progn
                (ros::ros-warn "distance-from-robot > 2000: ~A" world-pos)
                nil))))
    ;; else
    (progn
      (ros::ros-warn "Failed to transform /base_footprint to ~A" (send msg :header :frame_id))
      nil))))

(defun people-cb (msg)
  (setq *people-look-at-pos* (people->look-at-pos msg)))

(subscribe
 "/people"
 people_msgs::People
 #'people-cb)

(provide :action-look-at-leg) ;; end of action-look-at-leg.l
