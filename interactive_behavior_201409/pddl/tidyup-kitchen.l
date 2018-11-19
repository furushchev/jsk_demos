#!/usr/bin/env roseus
;; tidyup-kitchen.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :new-room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

(defun run ()
  (setq userdata '(nil))
  (setq *pos-cleaner* nil)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :torso :waist-z :joint-angle 50)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (move-to 'start 'kitchen userdata)
  (find-object 'bottle 'kitchen userdata)
  (pick 'bottle 'larm 'kitchen userdata)
  (move-to 'kitchen 'fridge userdata)
  (open-door 'rarm 'fridge userdata)
  (place 'bottle 'larm 'fridge userdata)
  (close-door 'rarm 'fridge userdata)
  (move-to 'fridge 'cleaner userdata)
  (pick 'cleaner 'rarm 'cleaner userdata)
  (move-to 'cleaner 'kitchen-isle userdata)
  (clean-kitchen userdata)
  (move-to 'kitchen-isle 'cleaner userdata)
  (place 'cleaner 'rarm 'cleaner userdata))

(pr2-init)
