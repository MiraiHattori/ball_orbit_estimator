#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")

(defun target_publisher ()
  (ros::roseus "target_publisher")
  (ros::advertise "target_point"
                  geometry_msgs::PointStamped)

  (let (x y z vx vy vz rate msg)

   (setq x 6.0)
   (setq y 1.0)
   (setq z 0.0)
   (setq vx -8.0)
   (setq vy 0.3)
   (setq vz 3.3)
   (setq rate 100.0)
   (ros::rate rate)
   (while (>= z 0)

     (ros::spin-once)

     (setq x (+ x (/ vx rate)))
     (setq y (+ y (/ vy rate)))
     (setq z (+ z (/ vz rate) (/ (/ (/ -9.8 rate) rate) 2.0)))
     (setq vz (+ vz (/ -9.8 rate)))

     (setq msg (instance geometry_msgs::PointStamped :init
                         :point
                         (instance geometry_msgs::Point :init
                                   :x x
                                   :y y
                                   :z z)))
     (send msg :header :frame_id "map")
     (send msg :header :stamp (ros::time-now))

     (print "target")
     (format t "x: ~A   y: ~A   z: ~A~%" x y z)

     (ros::publish "target_point" msg)

     (ros::sleep))));; 10ms


(target_publisher)
