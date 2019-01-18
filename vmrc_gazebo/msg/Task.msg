# Task name.
string name

# Task state.
string state

# Absolute sim time when this task started (ROS time).
time start_time

# Time elapsed since task started (ROS time). This is the current sim time
# minus start time.
# When the elapsed time reaches the task timeout, `timed_out` is set to true.
duration elapsed_time

# Remaining time until the task times out (ROS time).
duration remaining_time

# True if task timed out.
bool timed_out

# True if task has finished.
bool finished

# The score.
float64 score
