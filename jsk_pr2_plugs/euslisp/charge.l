;; charge.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")

(defparameter *detection-topic* "/narrow_stereo/left/ObjectDetection")
(defparameter *outlet* nil)

(defun charge-init ()
  (setq *scene* (make-eng2-scene))
  (pr2-init))

(defun go-to-charge-spot ()
  (pr2-tuckarm-pose)
  (move-to-spot "/eng2/7f/room73B2-charge-spot1")
  (pr2-reset-pose)
  )

(defun look-at-outlet (&key (dp 0.0) (dy 0.0))
  (send *pr2* :head :neck-p :joint-angle (+ 20.0 dp))
;;  (send *pr2* :head :neck-y :joint-angle (+ -117.671 dy))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun detect-outlet ()
  (let ((dp 0.0)
        (dy 0.0))
    ;; detect outlet
    (dotimes (i 5)
      (look-at-outlet :dp dp :dy dy)
      (setq *outlet*
            (check-detection :type "outlet"
                             :speak-name "konsento"
                             :timeout 20
                             :speak t))
      (when *outlet*
        (return-from detect-outlet t))
      ;; look around on failure
      (setq dp (- (random 10.0) 5.0))
      (setq dy (- (random 10.0) 5.0)))))



(defun pick-plug ()
  ;; pre-pick pose
  (send *ri* :angle-vector-sequence
        (list
         (float-vector 49.9702 60.0137 73.793 70.1833 -119.999 20.0076 -29.9992 179.999 -89.7222 -19.6339 -76.2008 -102.132 -49.1973 -61.4002 196.509 2.1427 74.2702)
         (float-vector 49.9702 60.0137 73.7736 70.2016 -119.999 20.0076 -29.9992 179.999 -35.0331 13.9068 -107.733 -121.309 -127.129 -72.4935 216.995 2.1427 74.2702)
         (float-vector 49.9702 60.0184 73.7688 70.1924 -119.999 20.0076 -29.9992 179.999 -5.10214 39.8815 -131.419 -120.662 -138.683 -74.3606 224.905 2.1547 74.2702))
        (list 5000 5000 7000))
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector
         (float-vector 49.9702 60.0184 73.7688 70.1924 -119.999 20.0076 -29.9992 179.999 -4.73163 46.0759 -135.002 -110.078 -143.969 -75.0985 225.129 2.1307 74.2702) 4000)
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  ;; close gripper
  (send *ri* :move-gripper :rarm 0.0 :effort -1)  ;; arm-pos ~= 47

  ;; pick-up
  (send *pr2* :angle-vector
        (float-vector 49.9702 60.0184 73.7688 70.1924 -119.999 20.0076 -29.9992 179.999 -5.10214 39.8815 -131.419 -120.662 -138.683 -74.3606 224.905 2.1547 74.2702))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun connect-plug ()
  (send *pr2* :angle-vector
        (float-vector 50.1147 60.0137 73.6088 70.4773 -119.849 20.0274 -29.9223 180.063 -15.6761 39.0333 -118.152 -119.791 -141.076 -89.5623 298.477 2.1547 74.2702))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )


(defun connect-plug-old ()
  (send *pr2* :rotate pi/2 :z)
  (send *ri* :go-pos-unsafe 0 0 90)

;;  (send *pr2* :translate #f(0 -200 0) *pr2*)
;;  (send *ri* :go-pos-unsafe 0.0 -0.3)

  ;; move plug to outlet
  (let (avs (z-offset 100))
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
            (send *outlet* :copy-worldcoords)
            :translate (float-vector 0 0 z-offset) *outlet*)
           :rotation-axis t
           :use-torso t
           :look-at-target t)
     avs)
    (send *ri* :angle-vector-sequence
          (reverse avs) (list 3000 2000 4000))
    (send *ri* :wait-interpolation)

    (dotimes (i 5)
      ;; connect plug
      (send *pr2* :rarm :inverse-kinematics
            (send
             (send *outlet* :copy-worldcoords)
             :translate (float-vector 0 0 (- z-offset (* i 10))) *outlet*)
            :rotation-axis t
            :use-torso t
            :look-at-target t)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
      (send *ri* :wait-interpolation)))
  )


(defun charge ()
  (go-to-charge-spot)
  (pick-plug)
  (connect-plug)
  (pr2-reset-pose)
  (pr2-tuckarm-pose))

(provide :charge) ;; end of charge.l
