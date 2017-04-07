;; grasp-test.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")
(require :detection_interface "package://jsk_perception/euslisp/detection_interface.l")

(defparameter *detection-topic* "/kinect_head_c2/rgb/ObjectDetection")

;; prop
(setq target-name "room73b2-cup")
(setq arm :rarm)
;; end prop

;; move-to
(pr2-init)
#|
(pr2-pick-tray-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)
(send *ri* :go-pos 0.4 0 0)
|#

;; perception
(setq target
      (check-detection :type target-name
                       :speak nil))
(let ((cds (send target :copy-worldcoords)))
  (send target :newcoords (send *pr2* :copy-worldcoords))
  (send target :transform cds))

;; pick
(block pick
(block continue
  (unless
      (with-assoc-move-target
          (mt :move-target
              (send (send *pr2* arm :end-coords :copy-worldcoords)
                    :translate (float-vector 100 0 0)) ;; offset
              :parent-link (send *pr2* arm :end-coords :parent))
        (send *pr2* :inverse-kinematics
              ;; handle
              (make-coords :pos (v+ (send target :centroid)
                                    (float-vector 0 0 50)))
                    :move-target (car mt)
                    :rotation-axis :z
                    :link-list (send *pr2* :link-list (send (car mt) :parent))))
    (ros::ros-error "no ik found")
    (return-from continue))
  ;; pre pick pose
  (send *pr2* :look-at-target (send *pr2* arm :end-coords))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;; open gripper
  (send *ri* :stop-grasp arm :wait t)
  (unix:sleep 8) ;; omajinai
  ;; reach to target object
  (send *pr2* arm :move-end-pos (float-vector 100 0 0))
  (send *pr2* :look-at-target (send *pr2* arm :end-coords))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (let (end-coords-before-pick diff-pose)
    (unwind-protect
         (progn
           ;; close gripper
           (let ((gripper-pos
                   (send *ri* :start-grasp arm
                              :gain 0.1
                              :objects (send *ri* :find-object target))))
             (when (< gripper-pos 1.0)
               (ros::ros-warn "failed to grasp ~A < 1.0" gripper-pos)
               (return-from continue)))
           (send *pr2* arm :end-coords :assoc target)
           (setq end-coords-before-pick
                 (send *pr2* arm :end-coords :copy-worldcoords))
           ;; pick it up
           (send *pr2* arm :move-end-pos (float-vector 0 0 200) *pr2*)
           (send *pr2* :look-at-target (send *pr2* arm :end-coords))
           (send *ri* :wait-interpolation)
           ;; save as prop
           (send *pr2* :put arm t)
           (return-from pick t))
      (progn
        (unless (send *pr2* :get arm)
          (ros::ros-info "recover grasp failure")
          (when end-coords-before-pick
            (setq diff-pose
                  (send (send *pr2* arm :end-coords :copy-worldcoords)
                        :difference-position end-coords-before-pick))
            (when (> (norm diff-pose) 10)
              ;; revert arm pose
              (send *pr2* arm :move-end-pos diff-pose *pr2*)
              (send *ri* :angle-vector (send *pr2* :angle-vector))
              (send *ri* :wait-interpolation)))
          ;; open gripper
          (send *ri* :stop-grasp arm :wait t)
          (unix:sleep 8)
          (send *pr2* arm :end-coords :dissoc target)
          (send *pr2* arm :move-end-pos (float-vector -100 0 0))
          (send *ri* :angle-vector (send *pr2* :angle-vector))
          (send *ri* :wait-interpolation)))))))

