;; Auto-generated. Do not edit!


(when (boundp 'intera_core_msgs::AnalogIOStates)
  (if (not (find-package "INTERA_CORE_MSGS"))
    (make-package "INTERA_CORE_MSGS"))
  (shadow 'AnalogIOStates (find-package "INTERA_CORE_MSGS")))
(unless (find-package "INTERA_CORE_MSGS::ANALOGIOSTATES")
  (make-package "INTERA_CORE_MSGS::ANALOGIOSTATES"))

(in-package "ROS")
;;//! \htmlinclude AnalogIOStates.msg.html


(defclass intera_core_msgs::AnalogIOStates
  :super ros::object
  :slots (_names _states ))

(defmethod intera_core_msgs::AnalogIOStates
  (:init
   (&key
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:states __states) ())
    )
   (send-super :init)
   (setq _names __names)
   (setq _states __states)
   self)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:states
   (&rest __states)
   (if (keywordp (car __states))
       (send* _states __states)
     (progn
       (if __states (setq _states (car __states)))
       _states)))
  (:serialization-length
   ()
   (+
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ;; intera_core_msgs/AnalogIOState[] _states
    (apply #'+ (send-all _states :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; intera_core_msgs/AnalogIOState[] _states
     (write-long (length _states) s)
     (dolist (elem _states)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; intera_core_msgs/AnalogIOState[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (let (r) (dotimes (i n) (push (instance intera_core_msgs::AnalogIOState :init) r)) r))
     (dolist (elem- _states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get intera_core_msgs::AnalogIOStates :md5sum-) "0a05ba3bbb53a8a3b164e34946d619f7")
(setf (get intera_core_msgs::AnalogIOStates :datatype-) "intera_core_msgs/AnalogIOStates")
(setf (get intera_core_msgs::AnalogIOStates :definition-)
      "string[]         names
AnalogIOState[] states
================================================================================
MSG: intera_core_msgs/AnalogIOState
time timestamp
float64 value
bool isInputOnly

")



(provide :intera_core_msgs/AnalogIOStates "0a05ba3bbb53a8a3b164e34946d619f7")


