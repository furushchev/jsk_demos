;; action-look-at-face.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :action-disable "package://interactive_behavior_201409/euslisp/action-disable.l")

(ros::load-ros-manifest "opencv_apps")

(defparameter *face-look-at-pos* nil)
(defparameter *face-label* nil)
(defparameter *camera-model* nil)

(defun face-cb (msg)
  (when *camera-model*
    (setq *face-look-at-pos* (face->look-at-pos msg *camera-model*))))

(defun face->look-at-pos (msg cam)
  (let ((base->camera
          (send *tfl* :lookup-transform "/base_footprint"
                (send msg :header :frame_id) (send msg :header :stamp))))
    (unless base->camera
      (ros::ros-warn "Failed to transform /base_footprint to ~A" (send msg :header :frame_id))
      nil)
    ;; select image with label
    (let* ((faces (or (remove-if #'(lambda (f)
                                     (string= (send f :label :name) ""))
                                 (send msg :faces))
                      (send msg :faces)))
           ray-vec cds)
      (sort faces #'(lambda (x y)
                      (<= (* (send x :face :width) (send x :face :height))
                          (* (send y :face :width) (send y :face :height)))))
      (setq ray-vec
            (send cam :ray
                  (send (car faces) :face_center :x)
                  (send (car faces) :face_center :y)))
      (ros::ros-info "ray-vec: ~A" ray-vec)
      (setq cds (make-coords :pos ray-vec))
      (send cds :transform base->camera)
      (ros::ros-info "cds pos: ~A" (send cds :pos))
      (send cds :pos)
      )))

(let ((caminfo
        (one-shot-subscribe
         "/kinect_head_c2/rgb/camera_info"
         sensor_msgs::CameraInfo)))
  (setq *camera-model* (make-camera-from-ros-camera-info caminfo)))

(subscribe
"/face_recognition/output"
opencv_apps::FaceArrayStamped
#'face-cb)

(provide :action-look-at-face) ;; end of action-look-at-face.l
