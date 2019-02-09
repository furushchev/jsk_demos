#!/usr/bin/env roseus
;; sym.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :eus-pddl "package://pddl_planner/src/eus-pddl.l")


(defun cons-cdr (sexp)
  "(cons-cdr '(1 2 3 . 4)) -> 4"
  (and (consp sexp)
       (atom (cdr (last sexp)))
       (cdr (last sexp))))

(defun variable-p (sym)
  "Return T if symbol name starts with '?'"
  (and (symbolp sym)
       (= #\? (aref (string-downcase sym) 0))))

(defun replace-symbol (sexp sym rep &key (test #'equal))
  "Replace symbol in sexp to rep"
  (cond
   ((atom sexp)
    (if (funcall test sym sexp) rep sexp))
   ((cons-cdr sexp)
    (if (funcall test sym (car sexp))
        (cons rep (cdr sexp))
      (if (funcall test sym (cdr sexp))
          (cons (car sexp) rep) sexp)
      sexp))
   (t
    (mapcar #'(lambda (x) (replace-symbol x sym rep :test test)) sexp))))

(defun find-variables (sexp)
  (cond
   ((atom sexp)
    (if (variable-p sexp) (list sexp) nil))
   ((cons-cdr sexp)
    (append (remove-if-not #'variable-p (append sexp nil))
            (find-variables (cons-cdr sexp))))
   (t
    (remove-if-not #'identity
                   (remove-duplicates (flatten (mapcar #'find-variables sexp)))))))

(defun find-sexp (sexp &key (operator) (operand) (test #'equal))
  "Find sexp by key in sexp"
  (let (res)
    (dolist (ssexp sexp)
      ;; (warn "ssexp: ~A~%" ssexp)
      (cond
       ((or (eq (car ssexp) 'and)
            (eq (car ssexp) 'or))
        (setq res (append res
                          (find-sexp (cdr ssexp)
                                     :operator operator :operand operand :test test))))
       (t
        (when (or (and operator (funcall test (car ssexp) operator))
                  (and operand (find-if #'(lambda (y) (funcall test y operand))
                                        (cdr ssexp))))
          (setq res (append res (list ssexp)))))))
    res))

(defun remove-sexp (sexp subsexp &key (test #'equal))
  (if (funcall test sexp subsexp) nil
    (if (or (atom sexp) (cons-cdr sexp)) sexp
      (remove-if-not #'(lambda (x) (remove-sexp x subsexp :test test)) sexp))))

(defun replace-sexp (sexp subsexp rep &key (test #'equal))
  (append (remove-sexp sexp subsexp :test test)
          (list rep)))

(defun resolve-symbol (init-cond goal-cond sym rep &key (duplicate-variables t))
  "sym -> rep"
  (when (atom rep) (setq rep (list rep)))
  (let* ((init-tmpl (find-sexp init-cond :operand sym))
         (goal-tmpl (find-sexp goal-cond :operand sym))
         vars dup-var init-rep goal-rep)
    ;;
    (when duplicate-variables
      (setq vars (remove sym (find-variables (append init-tmpl goal-tmpl))))
      (dolist (var vars)
        (setq init-tmpl (append init-tmpl (find-sexp init-cond :operand var)))
        (setq goal-tmpl (append goal-tmpl (find-sexp goal-cond :operand var))))
      (setq init-tmpl (remove-duplicates init-tmpl :test #'equal))
      (setq goal-tmpl (remove-duplicates goal-tmpl :test #'equal)))
    ;;
    (dolist (srep rep)
      (setq init-rep (replace-symbol init-tmpl sym srep))
      (setq goal-rep (replace-symbol goal-tmpl sym srep))
      ;;
      (when duplicate-variables
        (dolist (var vars)
          (setq dup-var (read-from-string
                         (string-upcase (gensym (string-upcase var)))))
          (setq init-rep (replace-symbol init-rep var dup-var))
          (setq goal-rep (replace-symbol goal-rep var dup-var))))
      ;;
      (setq init-cond (append init-cond init-rep))
      (setq goal-cond (append goal-cond goal-rep)))
    ;;
    (dolist (s init-tmpl)
      (setq init-cond (remove-sexp init-cond s)))
    (dolist (s goal-tmpl)
      (setq goal-cond (remove-sexp goal-cond s)))
    ;;
    (list (cons :initial-condition init-cond)
          (cons :goal-condition goal-cond))))


(defun resolve-robot-at ()
  ;; TODO: at-updater
  'start)

(defun resolve-prefer (item who)
  (when (or (variable-p item) (variable-p who))
    (ros::ros-error "item=~A (~A) and who=~A (~A) must NOT be variable"
                    item (variable-p item) who (variable-p who))
    (return-from resolve-prefer nil))
  ;; TOOD: call service for bayesian
  (read-from-string (format nil "~A-FOR-~A" (string-upcase item) (string-upcase who))))

(defun resolve-usual-spot (item)
  ;; TODO: call prob loc map
  (list
   (cons (make-coords :pos #f(100 0 0)) 7)
   (cons (make-coords :pos #f(0 100 0)) 2)
   (cons (make-coords :pos #f(200 0 0)) 1)))

(defun resolve-usual-item (spot)
  ;; TODO: impl
  ;; アイテムごとに
  `(((on bowl ,spot) . 7)
    ((on cereal ,spot) . 2)
    ((on milk ,spot) . 1)))

(defun resolve-item-place (item cds)
  ;; TODO: inspect item with coords -> (in/on item spot)
  (cond
   ((eq 100.0 (elt (send cds :pos) 0))
    `(in ,item fridge))
   ((eq 100.0 (elt (send cds :pos) 1))
    `(on ,item counter))
   ((eq 200.0 (elt (send cds :pos) 0))
    `(on ,item table))))

(defun resolve-in-place (spot)
  ;; TODO: impl
  `((placable ,spot)
    (openable ,spot)))

(defclass pddl-situation-problem
  :super pddl-problem
  :slots ())
(defmethod pddl-situation-problem
  (:resolve (&key robot-at on onhand who)
    "resolve top-level function"
    ;; WARNING: the order is very severe
    ;;
    ;; resolve from initial observation
    (send self :resolve-robot-at robot-at)
    (send self :resolve-who who)
    (send self :resolve-on on)
    (send self :resolve-onhand)
    ;;
    ;; resolve using inst ops.
    (send self :resolve-prefer)
    (send self :resolve-usual)
    ;; (send self :resolve-in-place)
    t)
  ;;
  (:resolve-robot-at (&optional spot)
    (unless spot (setq spot (resolve-robot-at)))
    ;;
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (robot-at-sexp (find-sexp init-cond :operator 'robot-at)))
      (when (variable-p (cadar robot-at-sexp))
        (send self :initial-condition (replace-symbol init-cond (cadar robot-at-sexp) spot))
        (send self :goal-condition (replace-symbol goal-cond (cadar robot-at-sexp) spot)))))
  (:resolve-who (who)
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (prefer-sexp (append (find-sexp init-cond :operator 'prefer)
                                (find-sexp goal-cond :operator 'prefer)))
           (vars (remove-if-not #'variable-p (mapcar #'caddr prefer-sexp))))
      (dolist (var vars)
        (setq init-cond (replace-symbol init-cond var who))
        (setq goal-cond (replace-symbol goal-cond var who)))
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond)))
  ;;
  (:resolve-on (items)
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (spot (cadar (find-sexp init-cond :operator 'robot-at)))
           (vars (remove-if-not #'(lambda (x)
                                    (equal (car (last x)) spot))
                                (find-sexp init-cond :operator 'on)))
           rep)
      (dolist (var vars)
        (setq rep (resolve-symbol init-cond goal-cond
                                  (cadr var) items :duplicate-variables t))
        (setq init-cond (cdr (assoc :initial-condition rep)))
        (setq goal-cond (cdr (assoc :goal-condition rep))))
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond)))
  ;;
  (:resolve-onhand (&key rarm larm arms)
    ;; TODO: impl
    )
  ;;
  (:resolve-prefer ()
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (prefer-sexp (append (find-sexp init-cond :operator 'prefer)
                                (find-sexp goal-cond :operator 'prefer)))
           item who repitem)
      ;; check
      (unless prefer-sexp
        (ros::ros-info "no prefer operator found")
        (return-from :resolve-prefer nil))
      ;; resolve
      (dolist (sexp prefer-sexp)
        (setq item (cadr sexp) who (caddr sexp))
        (setq repitem (resolve-prefer item who))
        (setq init-cond (replace-symbol (remove-sexp init-cond sexp) item repitem))
        (setq goal-cond (replace-symbol (remove-sexp goal-cond sexp) item repitem)))
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond))) ;; resolve-prefer
  ;;
  (:resolve-usual ()
    (dotimes (i 2)
      (ros::ros-info "resolve-usual [~A]" i)
      (let* ((init-cond (copy-object (send self :initial-condition)))
             (goal-cond (copy-object (send self :goal-condition)))
             (usual-sexp (find-sexp (append init-cond goal-cond) :operator 'usual)))
        ;; check
        (unless usual-sexp
          (ros::ros-info "all usual operator was replaced")
          (return-from :resolve-usual t))
        (send self :resolve-usual-once)
        (pprint (send self :initial-condition))
        (pprint (send self :goal-condition))
        )))
  (:resolve-usual-once ()
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (usual-sexp (find-sexp (append init-cond goal-cond) :operator 'usual)))
      ;; check
      (unless usual-sexp
        (ros::ros-info "no usual operator found")
        (return-from :resolve-usual nil))
      (warn "usual: ~A~%" usual-sexp)
      ;; resolve
      (dolist (sexp usual-sexp)
        (unless (or (eq (length sexp) 3)
                    (eq (length sexp) 5))
          (ros::ros-error "invalid usage of USUAL: ~A" sexp)
          (ros::ros-error "USUAL func. must be length of 3 or 5")
          (return-from :resolve-usual nil))
        (let ((item (cadr sexp))
              (spot (caddr sexp))
              (stamp (if (eq (length sexp) 5) (elt sexp 4) nil)))
          (cond
           ((and (variable-p item)
                 (variable-p spot))
            (ros::ros-info "cannot resolve item=~A spot=~A" item spot))
           ;;
           ((variable-p item)
            (ros::ros-info "resolve usual using spot=~A" spot)
            (let ((ret (resolve-usual-item spot))
                  (init-tmpl (find-sexp init-cond :operand item))
                  (goal-tmpl (find-sexp goal-cond :operand item))
                  rep repinit repgoal)
              (ros::ros-warn "items: ~A" (mapcar #'(lambda (x) (cadr (car x))) ret))
              (setq rep (resolve-symbol init-cond goal-cond
                                        item (mapcar #'(lambda (x) (cadr (car x))) ret)))
              (setq init-cond (cdr (assoc :initial-condition rep)))
              (setq goal-cond (cdr (assoc :goal-condition rep)))
              (ros::ros-warn "~A" init-cond)
              (dolist (rep ret)
                (let* ((repsexp (car rep))
                       (repcost (cdr rep))
                       (reppred (car repsexp))
                       (repitem (cadr repsexp)))
                  ;; (ros::ros-warn "sexp: ~A item: ~A repitem: ~A" sexp item repitem)
                  ;; (send self :resolve-symbol item repitem :duplicate-variables t)
                  ;; (setq repinit (replace-symbol init-tmpl item repitem))
                  ;; (setq repgoal (replace-symbol goal-tmpl item repitem))
                  ;; ;; FIXME: reppred (on/in)
                  ;; (setq init-cond (append init-cond repinit))
                  ;; (setq goal-cond (append goal-cond repgoal))
                  ))
              (dolist (s init-tmpl)
                (setq init-cond (remove-sexp init-cond s)))
              (dolist (s goal-tmpl)
                (setq goal-cond (remove-sexp goal-cond s)))
              )) ;; (variable-p item)
           ;;
           ((variable-p spot)
            (ros::ros-info "resolve usual using item=~A" item)
            (let ((ret (resolve-usual-spot item))
                  (init-tmpl (find-sexp init-cond :operand spot))
                  (goal-tmpl (find-sexp goal-cond :operand spot))
                  repinit repgoal)
              ;;
              ;; (warn "~A ~A ~A ~A ~A~%" sexp item spot tmpl ret)
              (dolist (rep ret)
                ;; for all candidate (coords . cost) from probabilistic loc-map
                (let ((repsexp (resolve-item-place item (car rep)))
                      (repcost (cdr rep))
                      repsym repinittmpl repgoaltmpl)
                  (when repsexp
                    ;; when item is related to any spot
                    (setq repsym (car (last repsexp)))
                    (setq repinittmpl (replace-symbol init-tmpl spot repsym))
                    (setq repgoaltmpl (replace-symbol goal-tmpl spot repsym))
                    (setq repinit (remove-duplicates (append repinit repinittmpl) :test #'equal))
                    (setq repgoal (remove-duplicates (append repgoal repgoaltmpl) :test #'equal))
                    (setq repcost `((= (obj-cost ,item ,repsym) ,repcost)))
                    (setq init-cond (append init-cond repcost)))
                  )) ;; for cds
              ;;
              ;; group by or for all on/in
              (let ((reponin (append (find-sexp repinit :operator 'on)
                                     (find-sexp repinit :operator 'in))))
                (dolist (s reponin)
                  (setq repinit (remove-sexp repinit s)))
                (setq repinit (append repinit reponin))
                (when reponin
                  (setq init-cond (remove-duplicates
                                   (append init-cond repinit) :test #'equal))))
              ;;
              (let ((reponin (append (find-sexp repgoal :operator 'on)
                                     (find-sexp repgoal :operator 'in))))
                (dolist (s reponin)
                  (setq repgoal (remove-sexp repgoal s)))
                (setq repgoal (append repgoal `((or ,@reponin))))
                (when reponin
                  (setq goal-cond (remove-duplicates
                                   (append goal-cond repgoal) :test #'equal))))
              ;;
              (dolist (s init-tmpl)
                (setq init-cond (remove-sexp init-cond s)))
              (dolist (s goal-tmpl)
                (setq goal-cond (remove-sexp goal-cond s)))
              )) ;; (variable-p spot)
           (t (ros::ros-error "item=~A or spot=~A must be variable" item spot))))) ;; dolist
      ;;
      ;; remove converged usual
      (dolist (sexp (find-sexp init-cond :operator 'usual))
        (when (and (not (variable-p (cadr sexp)))
                   (not (variable-p (caddr sexp))))
          (setq init-cond (remove-sexp init-cond sexp))))
      (dolist (sexp (find-sexp goal-cond :operator 'usual))
        (when (and (not (variable-p (cadr sexp)))
                   (not (variable-p (caddr sexp))))
          (setq goal-cond (remove-sexp goal-cond sexp))))
      ;;
      ;; update conditions
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond)
      )) ;; resolve-usual
  ;;
  (:resolve-in-place ()
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (inplace-sexp (append (find-sexp init-cond :operator 'in-place)
                                 (find-sexp goal-cond :operator 'in-place)))
           spot repsexp)
      (dolist (ssexp inplace-sexp)
        (setq spot (cadr ssexp))
        (setq repsexp (resolve-in-place spot))
        (setq init-cond (append init-cond repsexp))
        (setq init-cond (remove-sexp init-cond ssexp))
        (setq goal-cond (remove-sexp goal-cond ssexp)))
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond))) ;; resolve-in-place
  ) ;; defmethod

(defun prob-tidyup ()
  (let ((prob (instance pddl-situation-problem :init :name 'hoge :domain 'fuga)))
    (send prob :metric '(minimize (total-cost)))
    (send prob :objects
          '((room73a1 . spot)
            (bookshelf . spot)
            (counter . spot)
            (kitchen . spot)
            (sink . spot)
            (drawer . spot)
            (dishwasher . spot)
            (fridge . spot)
            (table . spot)
            (start . spot)
            (book . item)
            (bowl . item)
            (bottle . item)
            (cup . item)
            (larm . arm)
            (rarm . arm)))
    (send prob :initial-condition
          '((in-place ?spot)
            (robot-at ?spot)
            (prefer ?obj ?who)
            (on ?obj ?spot)))
    (send prob :goal-condition
          '((in-place ?dest)
            (usual ?obj ?dest)
            (on ?obj ?dest)))
    prob))

(defun test-tidyup ()
  (let ((prob (instance pddl-situation-problem :init :name 'hoge :domain 'fuga)))
    (send prob :metric '(minimize (total-cost)))
    (send prob :objects
          '((room73a1 . spot)
            (bookshelf . spot)
            (counter . spot)
            (kitchen . spot)
            (sink . spot)
            (drawer . spot)
            (dishwasher . spot)
            (fridge . spot)
            (table . spot)
            (start . spot)
            (book . item)
            (bowl . item)
            (bottle . item)
            (cup . item)
            (larm . arm)
            (rarm . arm)))
    (send prob :initial-condition
          '((in-place ?spot)
            (robot-at ?spot)
            (prefer ?obj ?who)
            (on ?obj ?spot)))
    (send prob :goal-condition
          '((in-place ?dest)
            (usual ?obj ?dest)
            (on ?obj ?dest)))
    ;;
    (format t "~%")
    (format t "initial-condition=~%")
    (pprint (send prob :initial-condition))
    (format t "goal-condition=~%")
    (pprint (send prob :goal-condition))
    (send prob :resolve
          :robot-at 'kitchen
          :on '(cup tea bottle)
          :who 'furuta)
    (format t "~%")
    (format t "initial-condition=~%")
    (pprint (send prob :initial-condition))
    (format t "goal-condition=~%")
    (pprint (send prob :goal-condition))
    prob))

(defun test-prepare ()
  (let ((prob (instance pddl-situation-problem :init :name 'hoge :domain 'fuga)))
    (send prob :metric '(minimize (total-cost)))
    (send prob :objects
          '((room73a1 . spot)
            (bookshelf . spot)
            (counter . spot)
            (kitchen . spot)
            (sink . spot)
            (drawer . spot)
            (dishwasher . spot)
            (fridge . spot)
            (table . spot)
            (start . spot)
            (book . item)
            (bowl . item)
            (bottle . item)
            (cup . item)
            (larm . arm)
            (rarm . arm)))
    (send prob :initial-condition
          '((in-place ?spot)
            (robot-at start)
            (usual ?obj ?spot :before ?stamp)
            (on ?obj ?spot)))
    (send prob :goal-condition
          '((in-place table)
            (usual ?obj table :after ?stamp)
            (on ?obj table)))
    ;;
    (format t "~%")
    (format t "initial-condition=~%")
    (pprint (send prob :initial-condition))
    (format t "goal-condition=~%")
    (pprint (send prob :goal-condition))
    (send prob :resolve
          :robot-at 'kitchen
          :on '(cup tea bottle)
          :who 'furuta)
    (format t "~%")
    (format t "initial-condition=~%")
    (pprint (send prob :initial-condition))
    (format t "goal-condition=~%")
    (pprint (send prob :goal-condition))
    prob))


;;(test-tidyup)
;;(test-prepare)
