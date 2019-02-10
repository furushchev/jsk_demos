#!/usr/bin/env roseus
;; sym.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "mbtk")
(ros::load-ros-manifest "pr2_controllers_msgs")
(require :eus-pddl "package://pddl_planner/src/eus-pddl.l")
(require :semantic-world-client "package://semantic_world/src/semantic-world-client.l")
(require :new-room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

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
      (ros::ros-debug "ssexp: ~A" ssexp)
      (cond
       ((and
         (not (memq operator '(and or)))
         (or (eq (car ssexp) 'and)
             (eq (car ssexp) 'or)))
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


(defun resolve-robot-at (scene &key (threshold 400.0))
  (let ((spots (get-all-spots scene))
        (robot-pose (send *ri* :state :worldcoords))
        nearest dist)
    (sort spots #'<=
          #'(lambda (s)
              (norm (send robot-pose :difference-position (cdr s)))))
    (setq nearest (car spots)
          dist (norm (send robot-pose :difference-position (cdr nearest))))
    ;; (ros::ros-info "dist to ~A: ~A" (car nearest) dist)
    (unless (< dist threshold)
      (ros::ros-info "no near spot")
      (return-from resolve-robot-at 'start))
    (ros::ros-info "the robot is near ~A" (car nearest))
    ;; trim compound words
    (setq nearest
          (if (search "-" (car nearest))
              (subseq (car nearest) (1+ (search "-" (car nearest) :from-end t)))
            (car nearest)))
    ;; return as a symbol
    (read-from-string nearest)))


(defclass onhand-resolver
  :super propertied-object
  :slots (grasp-thre release-thre larm-state rarm-state))
(defmethod onhand-resolver
  (:init (grasp-max-dist release-min-dist)
    (setq grasp-thre grasp-max-dist
          release-thre release-min-dist))
  (:start ()
    (ros::subscribe "l_gripper_controller/state"
                    pr2_controllers_msgs::JointControllerState
                    #'send self :state-callback :larm)
    (ros::subscribe "r_gripper_controller/state"
                    pr2_controllers_msgs::JointControllerState
                    #'send self :state-callback :rarm))
  (:stop ()
    (ros::unsubscribe "l_gripper_controller/state")
    (ros::unsubscribe "r_gripper_controller/state")
    (setq larm-state nil rarm-state nil)
    t)
  (:wait-for-result (&optional timeout)
    (when (send *ri* :simulation-modep) (return-from :wait-for-ready nil))
    (ros::rate 1)
    (let ((start-time (ros::time-now)))
      (while (ros::ok)
        (when (and larm-state rarm-state)
          (ros::ros-info "[onhand-resolver] :larm = ~A, :rarm = ~A" larm-state rarm-state)
          (return-from :wait-for-result t))
        (when (and timeout
                   (> (send (ros::time- (ros::time-now) start-time) :to-sec) timeout))
          (return-from :wait-for-result nil))
        (ros::spin-once)
        (ros::sleep))))
  (:state-callback (arm msg)
    (let ((actual (send msg :process_value))
          (desired (send msg :set_point))
          (err (send msg :error)))
      (cond
       ((and (< desired 0.0001)
             (> actual grasp-thre))
        (case arm
          (:larm (setq larm-state :onhand))
          (:rarm (setq rarm-state :onhand))))
       (t
        (case arm
          (:larm (setq larm-state :not-onhand))
          (:rarm (setq rarm-state :not-onhand)))))))
  (:get-result ()
    (list (cons :larm larm-state) (cons :rarm rarm-state))))

(defun resolve-onhand (&key (grasp-max-dist 0.005) (release-min-dist 0.01) (time-limit (* 60 60)))
  (let ((resolver (instance onhand-resolver :init grasp-max-dist release-min-dist))
        result retval)
    (send resolver :start)
    (unwind-protect
        (when (send resolver :wait-for-result)
          (setq result (send resolver :get-result)))
      (send resolver :stop))
    (when (or (eq (cdr (assoc :rarm result)) :onhand)
              (eq (cdr (assoc :larm result)) :onhand))
      (let* ((last-state (get-latest-state-from-db :after-time-from-now time-limit))
             (onhand-sexp (find-sexp last-state :operator 'onhand))
             arm-sexp)
        (setq arm-sexp (find-sexp onhand-sexp :operand 'larm))
        (when (and (eq (cdr (assoc :rarm result)) :onhand)
                   arm-sexp)
          (ros::ros-info "larm is grasping ~A" (cadar arm-sexp))
          (push (cons :larm (cadar arm-sexp)) retval))
        (setq arm-sexp (find-sexp onhand-sexp :operand 'rarm))
        (when (and (eq (cdr (assoc :larm result)) :onhand)
                   arm-sexp)
          (ros::ros-info "rarm is grasping ~A" (cadar arm-sexp))
          (push (cons :rarm (cadar arm-sexp)) retval))))
    retval))

(defun resolve-prefer (item who)
  (when (or (variable-p item) (variable-p who))
    (ros::ros-error "item=~A (~A) and who=~A (~A) must NOT be variable"
                    item (variable-p item) who (variable-p who))
    (return-from resolve-prefer nil))
  ;; TOOD: call service for bayesian
  ;; (read-from-string (format nil "~A-FOR-~A" (string-upcase item) (string-upcase who)))
  item)

(defclass usual-resolver
  :super propertied-object
  :slots (query-srv-name query-item world-frame-id weights cds-lst))
(defmethod usual-resolver
  (:init (item &key (srv-name "/ssd/query_position") (world "world"))
    (setq query-srv-name srv-name
          query-item item
          world-frame-id world))
  (:start ()
    (when (or (not (boundp '*tfl*)) (null *tfl*))
      (setq *tfl* (instance ros::transform-listener :init)))
    t)
  (:stop () t)
  (:wait-for-result (&optional (timeout 10))
    (unless (ros::wait-for-service query-srv-name timeout)
      (ros::ros-error "[usual-resolver] service not advertised: ~A" query-srv-name)
      (return-from :wait-for-result nil))
    (let (result)
      ;; query clustered item poses
      (setq result (ros::service-call query-srv-name
                                      (instance mbtk::QueryObjectPositionRequest :init
                                                :class_label (string-downcase query-item))))
      (when (null-string-p (send result :object_positions :header :frame_id))
        (ros::ros-error "[usual-resolver] no object position found for item ~A"
                        (string-downcase query-item))
        (return-from :wait-for-result nil))
      ;; transform for eus coords
      (unless (send *tfl* :wait-for-transform world-frame-id
                    (send result :object_positions :header :frame_id) (ros::time 0) 10)
        (ros::ros-error "[usual-resolver] failed to lookup transform: ~A ~A"
                          world-frame-id (send result :object_positions :header :frame_id))
        (return-from :wait-for-result nil))
      (setq weights (concatenate cons (send result :object_positions :weights)))
      (setq cds-lst (mapcar #'(lambda (pc)
                                (let ((ps (instance geometry_msgs::PoseStamped :init)))
                                  (send ps :header (send result :object_positions :header))
                                  (send ps :header :stamp (ros::time 0))
                                  (send ps :pose (send pc :pose))
                                  (send *tfl* :transform-pose world-frame-id ps)))
                            (send result :object_positions :poses)))
      t))
  (:get-result ()
    (list (cons :coords cds-lst) (cons :weights weights)))
  )

(defun resolve-usual-spot (item)
  (let ((resolver (instance usual-resolver :init item))
        result retval)
    (send resolver :start)
    (unwind-protect
        (when (send resolver :wait-for-result)
          (setq result (send resolver :get-result)))
      (send resolver :stop))
    (mapcar #'(lambda (c w) (cons c w))
            (cdr (assoc :coords result))
            (cdr (assoc :weights result)))))
  ;; (list
  ;;  (cons (make-coords :pos #f(100 0 0)) 7)
  ;;  (cons (make-coords :pos #f(0 100 0)) 2)
  ;;  (cons (make-coords :pos #f(200 0 0)) 1)))

(defun resolve-usual-item (spot)
  ;; TODO: impl
  ;; アイテムごとに
  `(
    ((on bowl ,spot) . 7)
    ((on cereal ,spot) . 2)
    ((on plasticbottle ,spot) . 1)
    ))

(defun resolve-item-place (item cds)
  (let ((sexp (cadr (assoc :states (inspect-item :coords cds :simplify t :single-in t)))))
    ;; FIXME: workaround for bug in semantic world
    (when (equal 'and (car sexp))
      (let ((s (find-sexp (list sexp) :operator 'in)))
        (setq sexp (if s (car s) (cadr sexp)))))
    ;;
    (replace-symbol sexp 'target item)))
#|
  (cond
   ((eq 100.0 (elt (send cds :pos) 0))
    `(in ,item fridge))
   ((eq 100.0 (elt (send cds :pos) 1))
    `(on ,item counter))
   ((eq 200.0 (elt (send cds :pos) 0))
    `(on ,item table))))
|#

(defun resolve-in-place (spot)
  ;; TODO: call service
  (let (retval)
    (push `(placable ,spot) retval)
    (when (memq spot '(fridge kitchen drawer))
      (push `(openable larm ,spot) retval)
      (push `(openable rarm ,spot) retval)
      )
    retval))

(defclass pddl-situation-problem
  :super pddl-problem
  :slots ())
(defmethod pddl-situation-problem
  ;;
  (:add-object (obj typ)
    (send self :objects
          (remove-duplicates
           (append (list (cons obj typ))
                   (send self :objects))
           :test #'equal)))
  ;;
  (:resolve (&key robot-at on who)
    "resolve top-level function"
    ;; WARNING: the order is very severe
    ;;
    ;; add arm objects
    (send self :add-object 'larm 'arm)
    (send self :add-object 'rarm 'arm)
    ;; resolve from initial observation
    (send self :resolve-robot-at robot-at)
    (send self :resolve-who who)
    (send self :resolve-on on)
    ;; (send self :resolve-onhand)
    ;;
    ;; resolve using inst ops.
    (send self :resolve-prefer)
    (send self :resolve-usual)
    (send self :resolve-in-place)
    (send self :append-costs)
    (send self :decompose-in)
    ;; add close door heuristics
    (send self :goal-condition
          (append (send self :goal-condition)
                  '((forall (?at - spot)
                     (not (open ?at))))))
    t)
  ;;
  (:resolve-robot-at (&optional spot)
    (ros::ros-info ">> [:resolve-robot-at] started")
    (unless spot (setq spot (resolve-robot-at)))
    ;;
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (robot-at-sexp (find-sexp init-cond :operator 'robot-at)))
      (unless (variable-p (cadar robot-at-sexp))
        (ros::ros-info "<< [:resolve-robot-at] No variable found")
        (return-from :resolve-robot-at t))
      (send self :initial-condition (replace-symbol init-cond (cadar robot-at-sexp) spot))
      (send self :goal-condition (replace-symbol goal-cond (cadar robot-at-sexp) spot))
      (send self :add-object spot 'spot)
      (ros::ros-info "<< [:resolve-robot-at] Replaced ~A=~A" (cadar robot-at-sexp) spot)
      t))
  ;;
  (:resolve-who (who)
    (ros::ros-info ">> [:resolve-who] started")
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (prefer-sexp (append (find-sexp init-cond :operator 'prefer)
                                (find-sexp goal-cond :operator 'prefer)))
           (vars (remove-if-not #'variable-p (mapcar #'caddr prefer-sexp))))
      (unless vars
        (ros::ros-info "<< [:resolve-who] No variable found")
        (return-from :resolve-who t))
      (dolist (var vars)
        (setq init-cond (replace-symbol init-cond var who))
        (setq goal-cond (replace-symbol goal-cond var who)))
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond)
      (send self :add-object who 'person)
      (ros::ros-info "<< [:resolve-who] Replaced ~A=~A" vars who)
      t))
  ;;
  (:resolve-on (items)
    (ros::ros-info ">> [:resolve-on] started")
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (spot (cadar (find-sexp init-cond :operator 'robot-at)))
           (vars (remove-if-not #'(lambda (x)
                                    (equal (car (last x)) spot))
                                (find-sexp init-cond :operator 'on)))
           rep)
      (unless vars
        (ros::ros-info "<< [:resolve-on] No variable found")
        (return-from :resolve-on t))
      (dolist (var vars)
        (setq rep (resolve-symbol init-cond goal-cond
                                  (cadr var) items :duplicate-variables t))
        (setq init-cond (cdr (assoc :initial-condition rep)))
        (setq goal-cond (cdr (assoc :goal-condition rep))))
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond)
      (dolist (item items)
        (send self :add-object item 'item))
      (ros::ros-info "<< [:resolve-on] Replaced ~A=~A" vars items)
      t))
  ;;
  (:resolve-onhand ()
    (ros::ros-info ">> [:resolve-onhand] started")
    (let ((init-cond (copy-object (send self :initial-condition)))
          (goal-cond (copy-object (send self :goal-condition)))
          (result (resolve-onhand))
          item)
      (when (cdr (assoc :larm result))
        (setq item (cdr (assoc :larm result)))
        (setq init-cond (append init-cond `((onhand ,item larm))))
        (ros::ros-info ">> [:resolve-onhand] larm=~A" item)
        (send self :add-object item 'item))
      (when (cdr (assoc :rarm result))
        (setq item (cdr (assoc :rarm result)))
        (setq init-cond (append init-cond `((onhand ,item rarm))))
        (ros::ros-info ">> [:resolve-onhand] rarm=~A" item)
        (send self :add-object item 'item))
      (send self :initial-condition init-cond)
      (ros::ros-info "<< [:resolve-onhand] end")
      t))
  ;;
  (:resolve-prefer ()
    (ros::ros-info ">> [:resolve-prefer] started")
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
        (setq goal-cond (replace-symbol (remove-sexp goal-cond sexp) item repitem))
        (send self :add-object item 'item)
        (ros::ros-info ">> [:resolve-prefer] replaced item  ~A=~A" item repitem))
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond)
      (ros::ros-info "<< [:resolve-prefer] end")
      t)) ;; resolve-prefer
  ;;
  (:resolve-usual ()
    (dotimes (i 5)
      (ros::ros-info ">> [:resolve-usual] started ~A" i)
      (let* ((init-cond (copy-object (send self :initial-condition)))
             (goal-cond (copy-object (send self :goal-condition)))
             (usual-sexp (find-sexp (append init-cond goal-cond) :operator 'usual)))
        ;; check
        (unless usual-sexp
          (ros::ros-info "<< [:resolve-usual] all usual operators ware resolved")
          (return-from :resolve-usual t))
        (send self :resolve-usual-once))))
  ;;
  (:resolve-usual-once ()
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (usual-sexp (find-sexp (append init-cond goal-cond) :operator 'usual)))
      ;; check
      (unless usual-sexp
        (ros::ros-info "<<< [:resolve-usual-once] no usual operator found")
        (return-from :resolve-usual nil))
      ;; resolve
      (dolist (sexp usual-sexp)
        (unless (or (eq (length sexp) 3)
                    (eq (length sexp) 5))
          (ros::ros-error "<<< [:resolve-usual-once] invalid usage of USUAL: ~A" sexp)
          (ros::ros-error "<<< [:resolve-usual-once] USUAL func. must be length of 3 or 5")
          (return-from :resolve-usual-once nil))
        (let ((item (cadr sexp))
              (spot (caddr sexp))
              (stamp (if (eq (length sexp) 5) (elt sexp 4) nil)))
          (cond
           ((and (variable-p item)
                 (variable-p spot))
            (ros::ros-info "<<< [:resolve-usual-once] cannot resolve item=~A spot=~A" item spot))
           ;;
           ((variable-p item)
            (ros::ros-info ">>> [:resolve-usual-once] resolve usual using spot=~A" spot)
            (let ((ret (resolve-usual-item spot))
                  (init-tmpl (find-sexp init-cond :operand item))
                  (goal-tmpl (find-sexp goal-cond :operand item))
                  repitems rep repinit repgoal)
              (setq repitems (mapcar #'(lambda (x) (cadr (car x))) ret))
              (setq rep (resolve-symbol init-cond goal-cond
                                        item repitems))
              (setq init-cond (cdr (assoc :initial-condition rep)))
              (setq goal-cond (cdr (assoc :goal-condition rep)))
              (dolist (s init-tmpl)
                (setq init-cond (remove-sexp init-cond s)))
              (dolist (s goal-tmpl)
                (setq goal-cond (remove-sexp goal-cond s)))
              (dolist (repitem repitems) (send self :add-object repitem 'item))
              )) ;; (variable-p item)
           ;;
           ((variable-p spot)
            (ros::ros-info ">>> [:resolve-usual-once] resolve usual using item=~A" item)
            (let ((ret (resolve-usual-spot item))
                  (init-tmpl (find-sexp init-cond :operand spot))
                  (goal-tmpl (find-sexp goal-cond :operand spot))
                  repinit repgoal repsyms)
              ;;
              (dolist (rep ret)
                ;; for all candidate (coords . cost) from probabilistic loc-map
                (let ((repsexp (resolve-item-place item (car rep)))
                      (repcost (cdr rep))
                      reppred repsym repinittmpl repgoaltmpl)
                  (when repsexp
                    ;; when item is related to any spot
                    (setq repsym (car (last repsexp)))
                    (unless (memq repsym repsyms)
                      (push repsym repsyms)
                      (send self :add-object repsym 'spot)
                      (ros::ros-debug "repsexp: ~A init-tmpl: ~A goal-tmpl: ~A"
                                     repsexp init-tmpl goal-tmpl)
                      (setq repinittmpl (replace-symbol init-tmpl spot repsym))
                      (setq repgoaltmpl (replace-symbol goal-tmpl spot repsym))
                      (setq reppred (car repsexp))
                      (setq repinittmpl (replace-symbol repinittmpl 'on reppred))
                      (setq repgoaltmpl (replace-symbol repgoaltmpl 'on reppred))
                      (ros::ros-debug "2 init-tmpl: ~A goal-tmpl: ~A" repinittmpl repgoaltmpl)
                      (setq repinit (remove-duplicates (append repinit repinittmpl) :test #'equal))
                      (setq repgoal (remove-duplicates (append repgoal repgoaltmpl) :test #'equal))
                      ;; coeff for obj-cost
                      (setq repcost `((= (obj-cost ,item ,repsym) ,(- 1000 (floor (* 1000 repcost))))))
                      (setq init-cond (append init-cond repcost))))
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
           (t (ros::ros-error "<<< [:resolve-usual-once] item=~A or spot=~A must be variable" item spot))))) ;; dolist
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
      t)) ;; resolve-usual
  ;;
  (:resolve-in-place ()
    (ros::ros-info ">> [:resolve-in-place] started")
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
        (setq goal-cond (remove-sexp goal-cond ssexp))
        (ros::ros-info ">> [:resolve-in-place] resolved ~A" spot))
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond)
      (ros::ros-info "<< [:resolve-in-place] end")
      t)) ;; resolve-in-place
  ;;
  (:append-costs (&key use-arms)
    (ros::ros-info ">> [:append-costs] started")
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (items (mapcar #'car (remove-if-not #'(lambda (x) (eq (cdr x) 'item))
                                               (send self :objects))))
           (spots (mapcar #'car (remove-if-not #'(lambda (x) (eq (cdr x) 'spot))
                                               (send self :objects))))
           )
      ;; arm cost
      (push '(= (arm-cost larm) 10) init-cond)
      (push '(= (arm-cost rarm) 10) init-cond)
      (when use-arms (push '(= (arm-cost arms) 30) init-cond))

      ;; obj cost
      (let ((unique-items (remove-duplicates items))
            exist-spots unique-spots)
        (dolist (item (append unique-items (list 'cleaner)))
          (setq exist-spots (mapcar #'(lambda (x) (car (last (cadr x))))
                                    (remove-if-not
                                     #'(lambda (x) (and (eq (car x) '=)
                                                        (eq (caadr x) 'obj-cost)
                                                        (eq (cadr (cadr x)) item)))
                                     init-cond)))
          (dolist (spot (set-difference spots exist-spots))
            (push `(= (obj-cost ,item ,spot) 1000) init-cond))))

      ;; move cost
      (let ((unique-spots (remove-duplicates spots)))
        (setq init-cond (append init-cond (make-spot-distance
                                           (append unique-spots (list 'somewhere))))))

      ;; other costs
      (push '(= (total-cost) 0) init-cond)
      (push '(= (manip-cost) 10) init-cond)
      (push '(= (find-cost) 50) init-cond)
      ;;
      (send self :initial-condition init-cond)
      (ros::ros-info ">> [:append-costs] end")
      t)) ;; append-costs
  ;;
  (:decompose-in ()
    (ros::ros-info ">> [:decompose-in] started")
    (let* ((init-cond (copy-object (send self :initial-condition)))
           (goal-cond (copy-object (send self :goal-condition)))
           (goal-or-insexp (remove-if-not #'(lambda (x)
                                              (find-sexp (list x) :operator 'in))
                                          (find-sexp goal-cond :operator 'or)))
           init-insexp goal-insexp in-spots)
      ;; remove in from sexp
      ;; append on spot-in sexp
      ;; add spot-in to objects
      (dolist (or-sexp goal-or-insexp)
        (let (rep)
          (dolist (sexp (cdr or-sexp))
            (let ((subrep (copy-object sexp)) spot repspot)
              (setq spot (car (last sexp))
                    repspot (read-from-string (string-upcase (format nil "~A-in" spot))))
              (send self :add-object repspot 'spot)
              (push (caddr sexp) in-spots)
              (setq subrep (replace-symbol subrep spot repspot))
              (setq subrep (replace-symbol subrep 'in 'on))
              (push subrep rep)))
          (setq goal-cond (remove-sexp goal-cond or-sexp))
          (push `(or ,@rep) goal-cond)))
      ;;
      (setq init-insexp (find-sexp init-cond :operator 'in))
      (setq goal-insexp (find-sexp goal-cond :operator 'in))
      ;;
      (dolist (sexp init-insexp)
        (setq init-cond (remove-sexp init-cond sexp))
        (push (caddr sexp) in-spots)
        (setq repspot
              (read-from-string (string-upcase (format nil "~A-in" (caddr sexp)))))
        (push `(on ,(cadr sexp) ,repspot) init-cond)
        (send self :add-object repspot 'spot))
      (dolist (sexp goal-insexp)
        (setq goal-cond (remove-sexp goal-cond sexp))
        (push (caddr sexp) in-spots)
        (setq repspot
              (read-from-string (string-upcase (format nil "~A-in" (caddr sexp)))))
        (push `(on ,(cadr sexp) ,repspot) goal-cond)
        (send self :add-object repspot 'spot))
      ;; remap openable operand
      (let* ((openable-sexp (find-sexp init-cond :operator 'openable))
             (repsexp (copy-object openable-sexp))
             (spots (remove-duplicates
                     (mapcar #'caddr openable-sexp)))
             repspot)
        (dolist (sexp openable-sexp)
          (setq init-cond (remove-sexp init-cond sexp))
          (setq repspot
                (read-from-string (string-upcase (format nil "~A-in" (caddr sexp)))))
          (setq repsexp (replace-symbol repsexp (caddr sexp) repspot)))
        (setq init-cond (append init-cond repsexp)))
      ;; duplicate obj-costs
      (let ((obj-cost-sexp
             (remove-if-not #'(lambda (x)
                                (eq (caadr x) 'obj-cost))
                            (find-sexp init-cond :operator '=)))
            spot repspot)
        (dolist (sexp obj-cost-sexp)
          (setq spot (car (last (cadr sexp))))
          (when (memq spot in-spots)
            (setq repspot
                  (read-from-string (string-upcase (format nil "~A-in" spot))))
            (push (replace-symbol sexp spot repspot) init-cond))))
      ;; append move-costs for spot-in
      (let ((move-cost-sexp
             (remove-if-not #'(lambda (x)
                                (eq (caadr x) 'move-cost))
                            (find-sexp init-cond :operator '=)))
            from to cost from-in to-in)
        (dolist (sexp move-cost-sexp)
          (setq from (cadr (cadr sexp)) to (caddr (cadr sexp))
                cost (car (last sexp))
                from-in
                (read-from-string (string-upcase (format nil "~A-in" from)))
                to-in
                (read-from-string (string-upcase (format nil "~A-in" to))))
          (cond
           ((and (memq from in-spots)
                 (memq to in-spots))
            (push `(= (move-cost ,from ,to-in) ,cost) init-cond)
            (push `(= (move-cost ,from-in ,to) ,cost) init-cond)
            (push `(= (move-cost ,from-in ,to-in) ,cost) init-cond))
           ((memq from in-spots)
            (push `(= (move-cost ,from-in ,to) ,cost) init-cond))
           ((memq to in-spots)
            (push `(= (move-cost ,from ,to-in) ,cost) init-cond)))))
      ;;
      (send self :initial-condition init-cond)
      (send self :goal-condition goal-cond)
      (ros::ros-info "<< [:decompose-in] end")
      t)) ;; decompose-in
  ) ;; defmethod

(defun prob-tidyup ()
  (let ((prob (instance pddl-situation-problem :init :name 'hoge :domain 'room-domain)))
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
  (let ((prob (instance pddl-situation-problem :init :name 'hoge :domain 'room-domain)))
    (send prob :metric '(minimize (total-cost)))
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
          :on '(cup spoon plasticbottle)
          :who 'furuta)
    (format t "~%")
    (format t "initial-condition=~%")
    (pprint (send prob :initial-condition))
    (format t "goal-condition=~%")
    (pprint (send prob :goal-condition))
    prob))

(defun test-prepare ()
  (let ((prob (instance pddl-situation-problem :init :name 'hoge :domain 'room-domain)))
    (send prob :metric '(minimize (total-cost)))
    (send prob :objects
          '(
            (table . spot)
            (start . spot)))
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
          :who 'furuta)
    (format t "~%")
    (format t "initial-condition=~%")
    (pprint (send prob :initial-condition))
    (format t "goal-condition=~%")
    (pprint (send prob :goal-condition))
    prob))

(defun test-prepare2 ()
  (let ((prob (instance pddl-situation-problem :init :name 'hoge :domain 'room-domain)))
    (send prob :metric '(minimize (total-cost)))
    (send prob :objects
          '(
            (fridge . spot)
            (fridge-in . spot)
            (kitchen . spot)
            (kitchen-in . spot)
            (plasticbottle . item)
            (table . spot)
            (start . spot)
            (larm . arm)
            (rarm . arm)))
    (send prob :initial-condition
          '((= (find-cost) 50)
            (= (manip-cost) 10)
            (= (total-cost) 0)
            (= (obj-cost cleaner start) 1000)
            (= (obj-cost cleaner table) 1000)
            (= (obj-cost cleaner kitchen) 1000)
            (= (obj-cost cleaner kitchen-in) 1000)
            (= (obj-cost cleaner fridge) 1000)
            (= (obj-cost cleaner fridge-in) 1000)
            (= (obj-cost plasticbottle start) 1000)
            (= (obj-cost plasticbottle table) 1000)
            (= (arm-cost rarm) 10)
            (= (arm-cost larm) 10)
            (robot-at start)
            (= (obj-cost plasticbottle kitchen) 767)
            (= (obj-cost plasticbottle kitchen-in) 767)
            (= (obj-cost plasticbottle fridge) 801)
            (= (obj-cost plasticbottle fridge-in) 801)
            (on plasticbottle kitchen)
            (on plasticbottle fridge-in)
            (openable rarm kitchen-in)
            (openable larm kitchen-in)
            (placable kitchen)
            (openable rarm fridge-in)
            (openable larm fridge-in)
            (placable fridge)
            (placable table)
            (= (move-cost somewhere somewhere) 1000)
            (= (move-cost start start) 1000)
            (= (move-cost table table) 1000)
            (= (move-cost kitchen kitchen) 1000)
            (= (move-cost kitchen kitchen-in) 1000)
            (= (move-cost kitchen-in kitchen) 1000)
            (= (move-cost kitchen-in kitchen-in) 1000)
            (= (move-cost fridge fridge) 1000)
            (= (move-cost fridge-in fridge) 1000)
            (= (move-cost fridge fridge-in) 1000)
            (= (move-cost fridge-in fridge-in) 1000)
            (= (move-cost somewhere start) 30)
            (= (move-cost somewhere table) 30)
            (= (move-cost somewhere kitchen) 30)
            (= (move-cost somewhere kitchen-in) 30)
            (= (move-cost somewhere fridge) 30)
            (= (move-cost somewhere fridge-in) 30)
            (= (move-cost start somewhere) 30)
            (= (move-cost start table) 24)
            (= (move-cost start kitchen) 30)
            (= (move-cost start kitchen-in) 30)
            (= (move-cost start fridge) 25)
            (= (move-cost start fridge-in) 25)
            (= (move-cost table somewhere) 30)
            (= (move-cost table start) 24)
            (= (move-cost table kitchen) 30)
            (= (move-cost table kitchen-in) 30)
            (= (move-cost table fridge) 0)
            (= (move-cost table fridge-in) 0)
            (= (move-cost kitchen somewhere) 30)
            (= (move-cost kitchen start) 30)
            (= (move-cost kitchen table) 30)
            (= (move-cost kitchen fridge) 30)
            (= (move-cost kitchen fridge-in) 30)
            (= (move-cost kitchen-in somewhere) 30)
            (= (move-cost kitchen-in start) 30)
            (= (move-cost kitchen-in table) 30)
            (= (move-cost kitchen-in fridge) 30)
            (= (move-cost kitchen-in fridge-in) 30)
            (= (move-cost fridge somewhere) 30)
            (= (move-cost fridge start) 25)
            (= (move-cost fridge table) 0)
            (= (move-cost fridge kitchen) 30)
            (= (move-cost fridge kitchen-in) 30)
            (= (move-cost fridge-in somewhere) 30)
            (= (move-cost fridge-in start) 25)
            (= (move-cost fridge-in table) 0)
            (= (move-cost fridge-in kitchen) 30)
            (= (move-cost fridge-in kitchen-in) 30)
            ))
    (send prob :goal-condition
          '((on plasticbottle table)))
    (pr2-init)
    (plan-task (make-domain) prob :timeout 30 :update nil)))

;; NEED TO RUN BELOW!
;;
;; export ROBOT=sim
;; roslaunch pddl_planner pddl_downward.launch
;; roslaunch mbtk visualize_log_eng8.launch use_mock:=false rviz:=false
;; roslaunch semantic_world semantic_world_user.launch
;;

;;(setq prob (test-tidyup))
(setq prob (test-prepare))
(setq dom (make-domain))
(setq sm (plan-task dom prob :update nil :timeout 3))
(setq insp (instance state-machine-inspector :init sm))
(send insp :publish-structure)
(send insp :publish-all-status)
;;(setq sm (test-prepare2))
