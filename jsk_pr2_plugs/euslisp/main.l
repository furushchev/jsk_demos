;; main.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")

(defparameter *detection-topic* "/narrow_stereo/left/ObjectDetection")

(require :charge "charge.l")

(defun init ()
  (charge-init))

(init)
(warn ";; charge")