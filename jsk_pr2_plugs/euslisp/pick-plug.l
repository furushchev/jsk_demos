(require :pr2-interface "package://pr2eus/pr2-interface.l")

(pr2-init)

(setq plug (make-cube 45 45 40))
(send plug :rotate pi :y)
(send plug :translate (float-vector 100 0 (+ 280 20)) *pr2*)

(send *pr2* :inverse-kinematics plug
      :move-target (send *pr2* :l_forearm_cam)
;;      :debug-view :no-message
      :translation-axis :z
      :rotation-axis :z
      :rthre (deg2rad 45))

(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)
