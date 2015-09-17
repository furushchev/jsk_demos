;; main.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")

(defun init ()
  (pr2-init))

(defun go-to-charge-spot ()
  (pr2-tuckarm-pose)
  (move-to-spot "eng2/7f/room73B2-charge-spot1")
  )

(defun look-at-drawer ()
  (pr2-reset-pose)
  (send *pr2* :head :neck-p :joint-angle 40)
  (send *pr2* :head :neck-y :joint-angle 90)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun pre-pick-plug-pose ()
  (send *ri* :angle-vector-sequence
        (list
         #f(49.9702 60.0137 73.793 70.1833 -119.999 20.0076 -29.9992 179.999 -89.7222 -19.6339 -76.2008 -102.132 -49.1973 -61.4002 196.509 2.1427 74.2702)
         #f(49.9702 60.0137 73.7736 70.2016 -119.999 20.0076 -29.9992 179.999 -35.0331 13.9068 -107.733 -121.309 -127.129 -72.4935 216.995 2.1427 74.2702)
         #f(49.9702 60.0184 73.7688 70.1924 -119.999 20.0076 -29.9992 179.999 -5.10214 39.8815 -131.419 -120.662 -138.683 -74.3606 224.905 2.1547 74.2702)
         #f(49.9702 60.0184 73.7688 70.1924 -119.999 20.0076 -29.9992 179.999 -4.73163 46.0759 -135.002 -110.078 -143.969 -75.0985 225.129 2.1307 74.2702))
        (list 3000 3000 3000 3000))
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector
        #f(50.1472 56.7978 73.4489 73.6746 -121.276 20.1799 -29.8598 180.009 -122.342 1.76042 -188.024 -76.642 -72.7825 -5.72958 193.143 -117.671 26.034)
        7000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector
        #f(49.9702 60.0184 73.7688 70.1924 -119.999 20.0076 -29.9992 179.999 -5.10214 39.8815 -131.419 -120.662 -138.683 -74.3606 224.905 -117.671 26.034)
        3000)
  (send *ri* :wait-interpolation))

(defun post-pick-plug-pose ()
  (send *ri* :angle-vector-sequence
        (list
         #f(179.325 59.9472 73.6621 70.2659 -119.932 20.0109 -29.9969 179.996 -101.127 39.0963 -138.135 -91.896 -155.457 -47.187 285.061 2.1547 74.2702)
         #f(179.316 59.9804 73.5894 70.3119 -119.808 20.0109 -29.9371 179.996 -122.09 7.8094 -186.489 -85.816 -167.747 -5.72958 295.474 -117.719 59.466)
         #f(179.315 59.9852 73.5894 70.3119 -119.808 20.0109 -29.9371 179.996 -122.247 1.68287 -187.914 -76.5922 -72.6234 -5.72958 193.373 -117.647 59.466))
        (list 3000 3000 3000)))

(defun connect-plug ()
  (setq *detection-topic* "/narrow_stereo/left/ObjectDetection")
  (let ((cds
         (check-detection :type "outlet"
                          :speak-name "konsento"
                          :timeout 30
                          :speak t)))
    (when cds
      (send *ri* :ros-wait 0.0 :spin-self t)
      (send *pr2* :rarm :inverse-kinematics
            cds)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 3000))))


(provide :main) ;; end of main.l
