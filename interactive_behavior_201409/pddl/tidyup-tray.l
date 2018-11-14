#!/usr/bin/env roseus
;; tidyup-tray.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain :use-arms t))
  (setq *problem*
        (make-problem :init '((ROBOT-AT START)
                              (ON TRAY TABLE))
                      :goal '((ON TRAY KITCHEN))
                      :items '(TRAY)
                      :use-arms t
                      ))
  (setq *sm* (plan-task *domain* *problem* :debug debug))
  (setq *insp* (instance state-machine-inspector :init *sm*))
  (send *sm* :reset-state)
  (send *insp* :publish-structure)
  (send *insp* :publish-all-status)
  *sm*)

(defun run (&optional debug)
  (plan debug)
  (pprint (exec-preemptive-task *sm*)))

(defun make-graph ()
  (setq *domain* (make-domain :use-arms t))
  (setq *problem*
        (make-problem :init '((ROBOT-AT START)
                              (ON TRAY TABLE))
                      :goal '((ON TRAY KITCHEN))
                      :items '(TRAY)
                      :use-arms t
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
