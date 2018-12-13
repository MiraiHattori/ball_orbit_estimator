#!/usr/bin/env roseus

(ros::roseus-add-msgs "ball_state_msgs")

(defun target_publisher ()
  (ros::roseus "target_publisher")
  (ros::advertise "/pointgrey/estimated_ball_state"
                  ball_state_msgs::PosAndVelWithCovarianceStamped)

  (let (x y z vx vy vz rate msg)

   (setq x 5.0)
   (setq y 0.0)
   (setq z 0.0)
   (setq vx (* -5.5 0.975))
   (setq vy (* -5.5 (sqrt (- 1.0 (* 0.975 0.975)))))
   (setq vz 5.5)
   (setq rate 100.0)
   (ros::rate rate)
   (while (and (>= z 0) (ros::ok))

     (ros::spin-once)

     (setq x (+ x (/ vx rate)))
     (setq y (+ y (/ vy rate)))
     (setq z (+ z (/ vz rate) (/ (/ (/ -9.8 rate) rate) 2.0)))
     (setq vz (+ vz (/ -9.8 rate)))

     (setq msg (instance ball_state_msgs::PosAndVelWithCovarianceStamped :init
                         :point
                         (instance geometry_msgs::Point :init
                                   :x x
                                   :y y
                                   :z z)
                         :velocity
                         (instance geometry_msgs::Vector3 :init
                                   :x vx
                                   :y vy
                                   :z vz)
                         :pos_and_vel_covariance
                         #f(1 0 0 0 0 0
                            0 1 0 0 0 0
                            0 0 1 0 0 0
                            0 0 0 1 0 0
                            0 0 0 0 1 0
                            0 0 0 0 0 1)
                         ))
     (send msg :header :frame_id "ground")
     (send msg :header :stamp (ros::time-now))

     (print "ball_state")
     (format t "x: ~A   y: ~A   z: ~A~%" x y z)
     (setq ttc (- (/ y vy)))
     (setq x_pred (+ x (* vx ttc)))
     (setq y_pred (+ y (* vy ttc)))
     (setq z_pred (+ z (* vz ttc) (* -4.9 ttc ttc)))
     (format t "pred x: ~A   y: ~A   z: ~A ttc: ~A~%" x_pred y_pred z_pred ttc)
     (ros::publish "/pointgrey/estimated_ball_state" msg)
     (ros::sleep))));; 10ms




(target_publisher)
