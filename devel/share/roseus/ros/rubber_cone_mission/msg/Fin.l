;; Auto-generated. Do not edit!


(when (boundp 'rubber_cone_mission::Fin)
  (if (not (find-package "RUBBER_CONE_MISSION"))
    (make-package "RUBBER_CONE_MISSION"))
  (shadow 'Fin (find-package "RUBBER_CONE_MISSION")))
(unless (find-package "RUBBER_CONE_MISSION::FIN")
  (make-package "RUBBER_CONE_MISSION::FIN"))

(in-package "ROS")
;;//! \htmlinclude Fin.msg.html


(defclass rubber_cone_mission::Fin
  :super ros::object
  :slots (_num _centroid_x _centroid_y _aveD _minD _angle _size ))

(defmethod rubber_cone_mission::Fin
  (:init
   (&key
    ((:num __num) 0)
    ((:centroid_x __centroid_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:centroid_y __centroid_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:aveD __aveD) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:minD __minD) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:angle __angle) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:size __size) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _num (round __num))
   (setq _centroid_x __centroid_x)
   (setq _centroid_y __centroid_y)
   (setq _aveD __aveD)
   (setq _minD __minD)
   (setq _angle __angle)
   (setq _size __size)
   self)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:centroid_x
   (&optional __centroid_x)
   (if __centroid_x (setq _centroid_x __centroid_x)) _centroid_x)
  (:centroid_y
   (&optional __centroid_y)
   (if __centroid_y (setq _centroid_y __centroid_y)) _centroid_y)
  (:aveD
   (&optional __aveD)
   (if __aveD (setq _aveD __aveD)) _aveD)
  (:minD
   (&optional __minD)
   (if __minD (setq _minD __minD)) _minD)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:serialization-length
   ()
   (+
    ;; int64 _num
    8
    ;; float64[] _centroid_x
    (* 8    (length _centroid_x)) 4
    ;; float64[] _centroid_y
    (* 8    (length _centroid_y)) 4
    ;; float64[] _aveD
    (* 8    (length _aveD)) 4
    ;; float64[] _minD
    (* 8    (length _minD)) 4
    ;; float64[] _angle
    (* 8    (length _angle)) 4
    ;; float64[] _size
    (* 8    (length _size)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _num
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num) (= (length (_num . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num . bv) 0) 0) s)
              (write-long (ash (elt (_num . bv) 1) -1) s))
             ((and (class _num) (= (length (_num . bv)) 1)) ;; big1
              (write-long (elt (_num . bv) 0) s)
              (write-long (if (>= _num 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num s)(write-long (if (>= _num 0) 0 #xffffffff) s)))
     ;; float64[] _centroid_x
     (write-long (length _centroid_x) s)
     (dotimes (i (length _centroid_x))
       (sys::poke (elt _centroid_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _centroid_y
     (write-long (length _centroid_y) s)
     (dotimes (i (length _centroid_y))
       (sys::poke (elt _centroid_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _aveD
     (write-long (length _aveD) s)
     (dotimes (i (length _aveD))
       (sys::poke (elt _aveD i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _minD
     (write-long (length _minD) s)
     (dotimes (i (length _minD))
       (sys::poke (elt _minD i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _angle
     (write-long (length _angle) s)
     (dotimes (i (length _angle))
       (sys::poke (elt _angle i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _size
     (write-long (length _size) s)
     (dotimes (i (length _size))
       (sys::poke (elt _size i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _num
#+(or :alpha :irix6 :x86_64)
      (setf _num (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64[] _centroid_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _centroid_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _centroid_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _centroid_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _centroid_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _centroid_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _aveD
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _aveD (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _aveD i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _minD
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _minD (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _minD i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _angle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angle i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _size
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _size (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _size i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get rubber_cone_mission::Fin :md5sum-) "421d0d272a8d8a52e65daee64dcb6ee2")
(setf (get rubber_cone_mission::Fin :datatype-) "rubber_cone_mission/Fin")
(setf (get rubber_cone_mission::Fin :definition-)
      "int64 num            # 유효한 클러스터의 개수
float64[] centroid_x # 유효한 클러스터들의 x 좌표
float64[] centroid_y # 유효한 클러스터들의 y 좌표
float64[] aveD    # 각 클러스터의 중심점 x 좌표 리스트
float64[] minD    # 각 클러스터의 중심점 y 좌표 리스트
float64[] angle   # 각 클러스터의 각도 (필요한 경우)
float64[] size    # 각 클러스터의 크기 (필요한 경우)

")



(provide :rubber_cone_mission/Fin "421d0d272a8d8a52e65daee64dcb6ee2")


