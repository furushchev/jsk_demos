;; charge.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")

(defparameter *detection-topic* "/narrow_stereo/left/ObjectDetection")

(defun charge-init ()
  (setq *scene* (make-eng2-scene))
  (pr2-init))

(defun go-to-charge-spot ()
  (pr2-tuckarm-pose)
  (move-to-spot "eng2/7f/room73B2-charge-spot1")
  (pr2-reset-pose)
  )

(defun look-at-outlet (&key (dp 0.0) (dy 0.0))
  (send *pr2* :head :neck-p :joint-angle (+ 26 dp)
  (send *pr2* :head :neck-y :joint-angle (+ -117.671 dy))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun pick-plug ()
  ;; pre-pick pose
  (send *ri* :angle-vector-sequence
        (list
         #f(49.9702 60.0137 73.793 70.1833 -119.999 20.0076 -29.9992 179.999 -89.7222 -19.6339 -76.2008 -102.132 -49.1973 -61.4002 196.509 2.1427 74.2702)
         #f(49.9702 60.0137 73.7736 70.2016 -119.999 20.0076 -29.9992 179.999 -35.0331 13.9068 -107.733 -121.309 -127.129 -72.4935 216.995 2.1427 74.2702)
         #f(49.9702 60.0184 73.7688 70.1924 -119.999 20.0076 -29.9992 179.999 -5.10214 39.8815 -131.419 -120.662 -138.683 -74.3606 224.905 2.1547 74.2702)
         #f(49.9702 60.0184 73.7688 70.1924 -119.999 20.0076 -29.9992 179.999 -4.73163 46.0759 -135.002 -110.078 -143.969 -75.0985 225.129 2.1307 74.2702))
        (list 3000 3000 3000 3000))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector #f(49.9702 60.0184 73.7688 70.1924 -119.999 20.0076 -29.9992 179.999 -4.73163 46.0759 -135.002 -110.078 -143.969 -75.0985 225.129 2.1307 74.2702))

  ;; close gripper
  (send *ri* :move-gripper :rarm 0.0 :effort -1)  ;; arm-pos ~= 47

  ;; pick-up
  (send *pr2* :rarm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun connect-plug ()
  (let ((dp 0.0)
        (dy 0.0)
        outlet)
    ;; detect outlet
    (dotimes (i 5)
      (look-at-outlet :dp dp :dy dy)
      (setq outlet
            (check-detection :type "outlet"
                             :speak-name "konsento"
                             :timeout 10
                             :speak t))
      (when outlet
        (return))
      ;; look around on failure
      (setq dp (- (random 10.0) 5.0))
      (setq dy (- (random 10.0) 5.0)))

  ;; move plug to outlet
  (let (avs)
    (push
     (send *pr2* :rarm :inverse-kinematics
           (send 
            (send (send *pr2* :rarm :end-coords) :copy-worldcoords)
            :translate #f(100 -400 0) *pr2*)
           :use-torso t
           :look-at-target t)
     avs)
    (push
     (send *pr2* :rarm :inverse-kinematics
           (send 
            (send (send *pr2* :rarm :end-coords) :copy-worldcoords)
            :translate #f(-400 -150 0) *pr2*)
           :use-torso t
           :look-at-target t)
     avs)
    (push
     (send *pr2* :rarm :inverse-kinematics
           (send
            (send outlet :copy-worldcoords)
            :translate #f(0 0 -100) outlet)
           :rotation-axis :z
           :use-torso t
           :look-at-target t)
     avs)
    (send *ri* :angle-vector-sequence
          (reverse avs) (list 3000 2000 4000))
    (send *ri* :wait-interpolation))

  (dotimes (i 5)
    ;; connect plug
    (send *pr2* :rarm :inverse-kinematics
          (send
           (send outlet :copy-worldcoords)
           :translate #f(0 0 -70) outlet)
          :rotation-axis :z
          :use-torso t
          :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)

    ;; TODO: check plugged-in
    ;;;; - subscribe power_state
    ;;;; - move xy slightly

    (when plugged-in
        (return))
      (send *pr2* :rarm :inverse-kinematics
            (send
             (send outlet :copy-worldcoords)
             :translate #f(-300 0 -200) outlet)
            :rotation-axis :z
            :use-torso t
            :look-at-target t)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
      (send *ri* :wait-interpolation)

      ;; retry detect outlet
      (look-at-outlet :dp dp :dy dy)
      (setq outlet
            (check-detection :type "outlet"
                             :speak-name "konsento"
                             :timeout 10
                             :speak t))

      ;; pre connect pose
      (send *pr2* :rarm :inverse-kinematics
            (send
             (send outlet :copy-worldcoords)
             :translate #f(0 0 -100) outlet)
            :rotation-axis :z
            :use-torso t
            :look-at-target t)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
      (send *ri* :wait-interpolation)))

  ;; release plug
  (send *ri* :stop-grasp :rarm :wait t)
  (send *pr2* :rarm :inverse-kinematics
        (send
         (send outlet :copy-worldcoords)
         :translate #f(0 0 -200) outlet)
        :rotation-axis :z
        :use-torso t
        :look-at-target t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; TODO: switch on
  ))

(defun charge ()
  (go-to-charge-spot)
  (pick-plug)
  (connect-plug)
  (pr2-reset-pose)
  (pr2-tuckarm-pose))

(provide :charge) ;; end of charge.l
