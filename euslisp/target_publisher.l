#!/usr/bin/env roseus

(ros::roseus-add-msgs "ball_state_msgs")
(ros::roseus-add-msgs "visualization_msgs")


(defun hypot (x y)
  (sqrt (+ (* x x) (* y y))))
(defun target_publisher ()
  (ros::roseus "target_publisher")
  (ros::advertise "/pointgrey/estimated_ball_state"
                  ball_state_msgs::PosAndVelWithCovarianceStamped)
  (ros::advertise "/pointgrey/orbit_simulation"
                  visualization_msgs::MarkerArray)

  (let (x y z vx vy vz rate msg)

   (setq x 3.0)
   (setq y 4.0)
   (setq z 0.8)
   (setq vx -1.7)
   (setq vy -3.6)
   (setq vz 4.9)
   (setq rate 100.0)
   (ros::rate rate)
   (setq marker-id 0)
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
                         #f(0.01 0 0 0 0 0
                            0 0.01 0 0 0 0
                            0 0 0.01 0 0 0
                            0 0 0 0.01 0 0
                            0 0 0 0 0.01 0
                            0 0 0 0 0 0.01)
                         ))
     (send msg :header :frame_id "odom")
     (send msg :header :stamp (ros::time-now))

     (print "ball_state")
     (format t "x: ~A   y: ~A   z: ~A~%" x y z)
;     (setq ttc (/ (hypot x y) (hypot vx vy)))
;     (setq x_pred (+ x (* vx ttc)))
;     (setq y_pred (+ y (* vy ttc)))
;     (setq z_pred (+ z (* vz ttc) (* -4.9 ttc ttc)))
;     (format t "pred x: ~A   y: ~A   z: ~A ttc: ~A~%" x_pred y_pred z_pred ttc)
     (ros::publish "/pointgrey/estimated_ball_state" msg)
     (setq markers '())
     (setq sim-t 0.0)
     (setq sim-z z)
     (while (>= sim-z 0.0)
            (setq sim-x (+ x (* vx sim-t)))
            (setq sim-y (+ y (* vy sim-t)))
            (setq sim-z (+ z (* vz sim-t) (* -4.9 sim-t sim-t)))
            (setq sim-t (+ sim-t (/ 1.0 rate)))
            (setq marker (instance visualization_msgs::Marker :init))
            (send marker :header :frame_id "odom")
            (send marker :header :stamp (ros::time-now))
            (send marker :id (setq marker-id (+ marker-id 1)))
            (send marker :type 2) ;; this is SPHERE according to visualization_msgs/msg/Marker.msg
            (send marker :action 0)
            (send marker :lifetime (instance ros::time :init :nsec 10000))
            (setq pos (instance geometry_msgs::Point :init :x sim-x :y sim-y :z sim-z))
            (setq ori (instance geometry_msgs::Quaternion :init :w 1 :x 0 :y 0 :z 0))
            (send marker :pose (instance geometry_msgs::Pose :init :position pos :orientation ori))
            (send marker :scale (instance geometry_msgs::Vector3 :init :x 0.067 :y 0.067 :z 0.067))
            (send marker :color (instance std_msgs::ColorRGBA :init :r 1 :g 1 :b 1 :a 1))
            (push marker markers))
     ;; (reverse markers)
     (setq orbit (instance visualization_msgs::MarkerArray :init
                           :markers markers))
     (ros::publish "/pointgrey/orbit_simulation" orbit)
     (ros::sleep))));; 10ms




(target_publisher)
