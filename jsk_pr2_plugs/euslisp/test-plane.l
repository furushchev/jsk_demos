;; test-plane.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(load "models/arrow-object.l")

(setq a00 (arrow))
(setq a01 (arrow :pos #f(0 100 0)))
(setq a10 (arrow :pos #f(100 0 0)))
(setq a11 (arrow :pos #f(100 100 0)))
(setq cds (make-cascoords))
(send cds :assoc a00)
(send cds :assoc a01)
(send cds :assoc a10)
(send cds :assoc a11)
(setq plane (list a00 a01 a10 a11 (make-cube 10 10 10)))

(objects plane)
