;; action-disable.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defparameter *behavior-server-enabled* t)

(defparameter *subscribed-topics* nil)

(defmacro subscribe (name type cb)
  `(progn
     (ros::subscribe ,name ,type ,cb)
     (setq *subscribed-topics*
           (remove-if #'(lambda (x)
                          (string= (car x) ,name)) *subscribed-topics*))
     (push '(,name ,type ,cb) *subscribed-topics*)))

(defun unsubscribe (name)
  (ros::unsubscribe name)
  (setq *subscribed-topics*
        (remove name *subscribed-topics*
                :test #'(lambda (a b) (string= a (car b))))))

(defun resubscribe-all ()
  (dolist (x *subscribed-topics*)
    (ros::subscribe (car x) (cadr x) (caddr x))))

(defun unsubscribe-all ()
  (dolist (x *subscribed-topics*)
    (ros::unsubscribe (car x))))

(defun behavior-enable-cb (req)
  (setq *behavior-server-enabled* t)
  (ros::ros-info "behavior server enabled")
  (resubscribe-all)
  (send req :response))

(defun behavior-disable-cb (req)
  (setq *behavior-server-enabled* nil)
  (ros::ros-info "behavior server disabled")
  (unsubscribe-all)
  (send req :response))

(ros::advertise-service
 "/interactive_behavior_enable"
 std_srvs::Empty
 #'behavior-enable-cb)
(ros::advertise-service
 "/interactive_behavior_disable"
 std_srvs::Empty
 #'behavior-disable-cb)

(provide :action-disable) ;; end of action-disable.l
