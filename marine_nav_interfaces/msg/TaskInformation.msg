# Information about a navigation task to be accomplished.

# Tasks may be nested using slashes in 
# the id field. For example: A task named
# "survey_a" of type "survey" may have 
# sub-tasks of type "survey-line" named
# "survey_a/line_1", "survey_a/line_2",
# etc.
string                      id

# Type of task. Used to determine how to
# handle the task. For example, a "transit"
# task may aim to reach a goal position without
# prescibing a predefined path while a
# "survey_line" task specifies a path needing
# to be followed.
string                      type

# Lower number indicates higher priority
int16                       priority

# List of poses relevant to the task.
# Their exact meaning is task type specific.
geometry_msgs/PoseStamped[] poses

# YAML structure for extra task data. A speed
# member may be used for a survey or transit
# task.
string                      data

# YAML structure with task status information.
string                      status

# Flag to indicate task is completed.
bool                        done
