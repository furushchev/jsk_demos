;; publish_pcd_pose.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(ros::roseus "publish_pcd_pose")

(setq *topic-name* (format nil "~A/~A" (ros::get-name) "output"))
(setq *plane-topic* (format nil "~A/~A" (ros::get-name) "plane"))
(ros::advertise *topic-name* geometry_msgs::posestamped 1)
(ros::advertise *plane-topic* geometry_msgs::polygonstamped 1)

(defun coords->point (c)
  (instance geometry_msgs::point32 :init
            :x (* 0.001 (elt (send c :worldpos) 0))
            :y (* 0.001 (elt (send c :worldpos) 1))
            :z (* 0.001 (elt (send c :worldpos) 2))))

(defun publish-plane (header trans size)
  (let ((p00 (make-cube 10 10 10))
        (p10 (make-cube 10 10 10 :pos (float-vector size 0 0)))
        (p01 (make-cube 10 10 10 :pos (float-vector 0 size 0)))
        (p11 (make-cube 10 10 10 :pos (float-vector size size 0)))
        (cds (make-cascoords))
        (msg (instance geometry_msgs::polygonstamped :init :header header))
        pts)
    (setq pts (list p00 p10 p11 p01))
    (send cds :assoc p00)
    (send cds :assoc p10)
    (send cds :assoc p11)
    (send cds :assoc p01)
    (send cds :transform trans)
    (send msg :polygon :points
          (mapcar #'coords->point pts))
    (ros::publish *plane-topic* msg)))

(ros::rate 10)
(while (ros::ok)
 (let* ((cds (make-coords :pos (float-vector 100 100 0)
;;                          :rpy (list (deg2rad 45) 0 0)
                          :rpy (list 0 (deg2rad 45) (deg2rad 45))
                          ))
        (msg (ros::coords->tf-pose-stamped cds "base_link")))
   (ros::publish *topic-name* msg)
   (publish-plane (send msg :header) cds 1000)
   (ros::sleep)))

(exit)
