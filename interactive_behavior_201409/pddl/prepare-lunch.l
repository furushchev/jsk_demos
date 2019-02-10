#!/usr/bin/env roseus
;; prepare-lunch.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :new-room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

(defun make-graph ()
  (setq *domain* (make-domain))
  (setq *problem* (make-problem :init
                                '((ROBOT-AT START)
                                  (ON BOWL KITCHEN)
                                  (ON SPOON KITCHEN)
                                  (ON CUP SHELF)
                                  (AVAILABLE SHELF)
                                  )
                                :goal
                                '(
                                  (ON BOWL TABLE)
                                  (ON SPOON TABLE)
                                  (ON CUP TABLE)
                                  )
                                :items
                                '(CUP BOTTLE BOWL BOOK)
                                ))
  (setq failed-nodes nil)
  (unless failed-nodes
    (setq failed-nodes
          (remove-if-not #'(lambda (n)
                             (string= "_f"
                                      (subseq (send n :name)
                                              (- (length (send n :name)) 2))))
                         (send *domain* :action)))
    (setq failed-nodes
          (mapcar #'(lambda (n)
                      (read-from-string
                       (subseq (send n :name) 0 (- (length (send n :name)) 2))))
                  failed-nodes)))
  (setq gr (pddl-plan-to-graph nil
                               :domain *domain*
                               :problem *problem*
                               :failed-nodes failed-nodes
                               :readable t
                               :debug nil
                               :timeout 30))
  (send gr :write-to-pdf "/tmp/graph.pdf" nil "hoge"))

(pr2-init)
