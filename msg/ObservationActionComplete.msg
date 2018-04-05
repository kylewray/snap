# Observation message for when an action is completed or
# terminated early for any number of reasons.

# Which action was just completed, and optionally which
# action is next. This is usually in the form of
# the interrupting recovery and teleoperator 'actions'.
# This will likely be NONE in most other cases.
# ** This must be of the types found in ActionType.
uint8 completed_action
uint8 next_action

# Also record the action's termination result.
uint8 NONE = 0
uint8 SUCCESS = 1
uint8 FAILURE = 2
uint8 INTERRUPT = 3

uint8 result

