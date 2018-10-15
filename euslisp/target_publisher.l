#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")

(defun target_publisher ()
  (ros::roseus "target_publisher")
  (ros::advertise "target_point"
                  geometry_msgs::PointStamped)

  (let (theta rad x y z)

    (setq theta 0)

   (ros::rate 100)
   (while (ros::ok)

     (ros::spin-once)

     (setq theta (mod (+ theta 9) 360))
     (setq rad (deg2rad theta))

     (setq x 1)
     (setq y (+ -0.1 (* -0.4 (cos rad))))
     (setq z (+ 1.3 (* 0.4 (sin rad))))

     (setq msg (instance geometry_msgs::PointStamped :init
                         :point
                         (instance geometry_msgs::Point :init
                                   :x x
                                   :y y
                                   :z z)))
     (send msg :header :frame_id "origin")
     (send msg :header :stamp (ros::time-now))

     (print "target")
     (format t "x: ~A   y: ~A   z: ~A~%" x y z)

     (ros::publish "target_point" msg)

     (ros::sleep))));; 100ms


(target_publisher)
