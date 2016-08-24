;; publish_pcd_pose.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(ros::roseus "publish_pcd_pose")

(setq *topic-name* (format nil "~A/~A" (ros::get-name) "output"))
(ros::advertise *topic-name* geometry_msgs::posestamped 1)


(ros::rate 10)
(while (ros::ok)
 (let* ((cds (make-coords :pos (float-vector 100 100 0)
                          :rpy (list 0 (deg2rad 45) (deg2rad 45))))
        (msg (ros::coords->tf-pose-stamped cds "base_link")))
   (ros::publish *topic-name* msg)
   (ros::sleep)))

(exit)
