(ros::load-ros-manifest "elevator_move_base_pr2")
(ros::load-ros-manifest "dynamic_reconfigure")

(ros::roseus "elevator_settings")

(defun dynreconf-double (srv values)
  (let ((req (instance dynamic_reconfigure::reconfigurerequest :init)) res)
    (send req :config :doubles
          (mapcar #'(lambda (x)
                      (instance dynamic_reconfigure::doubleparameter :init
                                :name (car x) :value (cdr x)))
                  values))
    (setq res (ros::service-call (concatenate string srv "/set_parameters") req))
    (mapcar #'(lambda (x)
                (cons (send x :name) (send x :value)))
            (send res :config :doubles))))

(defun change-move-base-xy (val)
  (assoc "xy_goal_tolerance" (dynreconf-double "/move_base_node/DWAPlannerROS" (list (cons "xy_goal_tolerance" val)))
         :test #'string=))

(defun change-move-base-yaw (val)
  (assoc "yaw_goal_tolerance" (dynreconf-double "/move_base_node/DWAPlannerROS" (list (cons "yaw_goal_tolerance" val)))
         :test #'string=))

(defun change-template-match (name val)
  (let ((req (instance elevator_move_base_pr2::changematchingthresholdrequest :init
                       :name name :threshold val))
        res)
    (send (ros::service-call "/elevator_number/change_threshold" req) :ok)))

(defun change-sift-threshold (name rep dis err)
  (dynreconf-double (format nil "/narrow_stereo/left/point_pose_extractor_~A" name)
                    (list
                     (cons "reprojection_threshold" rep)
                     (cons "distanceratio_threshold" dis)
                     (cons "error_threshold" err))))
