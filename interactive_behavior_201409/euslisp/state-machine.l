;; state-machine.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(require :action-disable "package://interactive_behavior_201409/euslisp/action-disable.l")
(require :action-look-at-face "package://interactive_behavior_201409/euslisp/action-look-at-face.l")
;;(require :action-look-at-leg "package://interactive_behavior_201409/euslisp/action-look-at-leg.l")
;;(require :action-look-at-sound "package://interactive_behavior_201409/euslisp/action-look-at-sound.l")

(defun idle (&optional args)
  (ros::rate 5)
  (ros::spin-once)
  (unless *behavior-server-enabled*
    (return-from idle :disabled))
)

(defun sleep (&optional args)
  (unix:sleep 3)
  t)

(defun disabled (&optional args)
  (ros::rate 0.2)
  (while (and (ros::ok)
              (not *behavior-server-enabled*))
   (ros::spin-once)
   (ros::sleep))
  t)

(defun look-at-leg (&optional args)
  (when *people-look-at-pos*
    (look-at-pos *people-look-at-pos*)
    (setq *people-look-at-pos* nil))
  t)

(defun look-at-sound (&optional args)
  (when *sound-look-at-pos*
    (look-at-pos *sound-look-at-pos*)
    (setq *sound-look-at-pos* nil))
  t)

(defun look-at-face (&optional args)
  (when *face-look-at-pos*
    (look-at-pos *face-look-at-pos*)
    (setq *face-look-at-pos* nil))
  t)

(defun greet (&optional args)
  (ros::ros-info "greet ~A" args)
  t)

(defun make-behavior-state-machine ()
  (let ((sm (instance state-machine :init)))
    (dotimes (m '((:idle . 'idle)
                  (:end . 'end)
                  (:sleep . 'sleep)
                  (:disabled . 'disabled)
                  (:look-at-leg . 'look-at-leg)
                  (:look-at-sound . 'look-at-sound)
                  (:look-at-face . 'look-at-face)
                  (:greet . 'greet)))
      (send sm :add-node (instance state :init (car m) (cdr m))))
    (send sm :start-state :idle)
    (send sm :goal-state :end)

    ;; action transitions
    (dolist '(ac '(:disabled
                   :sleep
;;                   :look-at-sound
;;                   :look-at-leg
                   :look-at-face
                   ))
      (send sm :add-transition (send sm :node :idle) (send sm :node ac) ac)
      (send sm :add-transition (send sm :node ac) (send sm :node :idle) nil))

    ;; subaction transitions
    (send sm :add-transition (send sm :node :look-at-face) (send sm :node :greet) t)
    (send sm :add-transition (send sm :node :greet) (send sm :idle) t)

    sm))

(defun demo ()
  (setq *sm* (make-behavior-state-machine))
  (exec-smach-with-spin *sm*))

