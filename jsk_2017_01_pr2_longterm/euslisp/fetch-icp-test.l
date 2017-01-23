;; fetch-icp-test.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(require :fetch-interface "package://fetcheus/fetch-interface.l")

(fetch-init)

(send *fetch* :angle-vector (send *ri* :state :potentio-vector))

(send *fetch* :head :neck-p :joint-angle 40)
(send *ri* :angle-vector (send *fetch* :angle-vector))
(send *ri* :wait-interpolation)
