;; Auto-generated. Do not edit!


(when (boundp 'intera_motion_msgs::MotionCommandActionResult)
  (if (not (find-package "INTERA_MOTION_MSGS"))
    (make-package "INTERA_MOTION_MSGS"))
  (shadow 'MotionCommandActionResult (find-package "INTERA_MOTION_MSGS")))
(unless (find-package "INTERA_MOTION_MSGS::MOTIONCOMMANDACTIONRESULT")
  (make-package "INTERA_MOTION_MSGS::MOTIONCOMMANDACTIONRESULT"))

(in-package "ROS")
;;//! \htmlinclude MotionCommandActionResult.msg.html
(if (not (find-package "ACTIONLIB_MSGS"))
  (ros::roseus-add-msgs "actionlib_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass intera_motion_msgs::MotionCommandActionResult
  :super ros::object
  :slots (_header _status _result ))

(defmethod intera_motion_msgs::MotionCommandActionResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:status __status) (instance actionlib_msgs::GoalStatus :init))
    ((:result __result) (instance intera_motion_msgs::MotionCommandResult :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _status __status)
   (setq _result __result)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:result
   (&rest __result)
   (if (keywordp (car __result))
       (send* _result __result)
     (progn
       (if __result (setq _result (car __result)))
       _result)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; actionlib_msgs/GoalStatus _status
    (send _status :serialization-length)
    ;; intera_motion_msgs/MotionCommandResult _result
    (send _result :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; actionlib_msgs/GoalStatus _status
       (send _status :serialize s)
     ;; intera_motion_msgs/MotionCommandResult _result
       (send _result :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; actionlib_msgs/GoalStatus _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;; intera_motion_msgs/MotionCommandResult _result
     (send _result :deserialize buf ptr-) (incf ptr- (send _result :serialization-length))
   ;;
   self)
  )

(setf (get intera_motion_msgs::MotionCommandActionResult :md5sum-) "78437d84884cb2db646d41b73dbf98db")
(setf (get intera_motion_msgs::MotionCommandActionResult :datatype-) "intera_motion_msgs/MotionCommandActionResult")
(setf (get intera_motion_msgs::MotionCommandActionResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalStatus status
MotionCommandResult result

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: actionlib_msgs/GoalStatus
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text


================================================================================
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id


================================================================================
MSG: intera_motion_msgs/MotionCommandResult
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# result
bool result

string errorId
string FAILED_TO_PARAMETERIZE=FAILED_TO_PARAMETERIZE
string PLANNED_MOTION_COLLISION=PLANNED_MOTION_COLLISION
string INVALID_TRAJECTORY_MESSAGE=INVALID_TRAJECTORY_MESSAGE
string ENDPOINT_DOES_NOT_EXIST=ENDPOINT_DOES_NOT_EXIST
string CARTESIAN_INTERPOLATION_FAILED=CARTESIAN_INTERPOLATION_FAILED
string FINAL_POSE_NOT_WITHIN_TOLERANCE=FINAL_POSE_NOT_WITHIN_TOLERANCE
string CONTROLLER_NOT_FOLLOWING=CONTROLLER_NOT_FOLLOWING
string ZERO_G_ACTIVATED_DURING_TRAJECTORY=ZERO_G_ACTIVATED_DURING_TRAJECTORY
string PLANNED_JOINT_ACCEL_LIMIT=PLANNED_JOINT_ACCEL_LIMIT

TrajectoryAnalysis trajectory_analysis

int32 last_successful_waypoint
int32 HAVE_NOT_REACHED_FIRST_WAYPOINT=-1
int32 GENERAL_TRAJECTORY_FAILURE=-2


================================================================================
MSG: intera_motion_msgs/TrajectoryAnalysis
# The duration of the reference trajectory, as originally planned
float64 planned_duration

# The measured duration of the trajectory, as executed
float64 measured_duration

# Minimum commanded angle during trajectory for each joint
float64[] min_angle_command

# Maximum commanded angle during trajectory for each joint
float64[] max_angle_command

# Peak speed command = max(abs(reference velocity)) for each joint
float64[] peak_speed_command

# Peak accel command = max(abs(reference acceleration)) for each joint
float64[] peak_accel_command

# Peak jerk command = max(abs(reference jerk)) for each joint
float64[] peak_jerk_command

# Minimum trajectory time rate observed
float64 min_time_rate

# Maximium trajectory time rate observed
float64 max_time_rate

# Max joint position error = max(abs(position error)) for each joint
float64[] max_position_error

# Max joint velocity error = max(abs(velocity error)) for each joint
float64[] max_velocity_error

")



(provide :intera_motion_msgs/MotionCommandActionResult "78437d84884cb2db646d41b73dbf98db")


