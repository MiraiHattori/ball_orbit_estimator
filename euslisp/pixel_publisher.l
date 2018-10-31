#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "opencv_apps")

(defun target_publisher ()
  (ros::roseus "target_publisher")
  (ros::advertise "/pointgrey/ball_pixels"
                  opencv_apps::Point2DArrayStamped)

  (let (x y z vx vy vz rate msg)

   (setq x 6.0)
   (setq y 1.0)
   (setq z 0.0)
   (setq vx -8.0)
   (setq vy 0.3)
   (setq vz 3.3)
   (setq pl #2f((1738.808747 0.000000 832.927204 0.000000)
                (0.000000 1738.808747 542.995838 0.000000)
                (0.000000 0.000000 1.000000 0.000000)))
   (setq pr #2f((1738.808747 0.000000 832.927204 -158.386802)
                (0.000000 1738.808747 542.995838 0.000000)
                (0.000000 0.000000 1.000000 0.000000)))
   (setq rate 100.0)
   (ros::rate rate)
   (while (>= z 0)
     (ros::spin-once)

     (setq x (+ x (/ vx rate)))
     (setq y (+ y (/ vy rate)))
     (setq z (+ z (/ vz rate) (/ (/ (/ -9.8 rate) rate) 2.0)))
     (setq vz (+ vz (/ -9.8 rate)))
     (setq tmpl (transform pl (float-vector (- z) (- y) x 1)))
     (setq lx (/ (elt tmpl 0) (elt tmpl 2)))
     (setq ly (/ (elt tmpl 1) (elt tmpl 2)))
     (setq tmpr (transform pr (float-vector (- z) (- y) x 1)))
     (setq rx (/ (elt tmpr 0) (elt tmpr 2)))
     (setq ry (/ (elt tmpr 1) (elt tmpr 2)))

     (setq msg (instance opencv_apps::Point2DArrayStamped :init))
     (send msg :header :frame_id "map")
     (send msg :header :stamp (ros::time-now))
     (send msg :points (list (instance opencv_apps::Point2D :init :x lx :y ly)
                             (instance opencv_apps::Point2D :init :x rx :y ry)))

     (format t "left ~A:~A, right ~A:~A~%" lx ly rx ry)
     (when (and (>= lx 0) (< lx 1280) (>= ly 0) (< ly 1024) (>= rx 0) (< rx 1280) (>= ry 0) (< ry 1024))
         (print "publish")
         (ros::publish "/pointgrey/ball_pixels" msg))
     (ros::sleep))));; 10ms


(target_publisher)
