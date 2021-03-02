#ifndef AGVLIB_H
#define AGVLIB_H

enum ActionState {
    ACTION_FLOAT,
    ACTION_MANUAL,
    ACTION_INITIAL_POSE,
    ACTION_QUALITY_POSE,
    ACTION_ROTATE_GOAL,
    ACTION_MOVE_GOAL,
    ACTION_CHARGING_IN,
    ACTION_CHARGING_OUT,
    ACTION_LIFT_IN,
    ACTION_LIFT_UP,
    ACTION_LIFT_DOWN,
    ACTION_LIFT_OUT
};
 
enum State {
    STATE_EXECUTE,
    STATE_CANCEL,
    STATE_DONE
};

enum Status {
    STATUS_PENDING,        // = 0   # The goal has yet to be processed by the action server
    STATUS_ACTIVE,         // = 1   # The goal is currently being processed by the action server
    STATUS_PREEMPTED,      // = 2   # The goal received a cancel request after it started executing and has since completed its execution (Terminal State)
    STATUS_SUCCEEDED,      // = 3   # The goal was achieved successfully by the action server (Terminal State)
    STATUS_ABORTED,        // = 4   # The goal was aborted during execution by the action server due to some failure (Terminal State)
    STATUS_REJECTED,       // = 5   # The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)
    STATUS_PREEMPTING,     // = 6   # The goal received a cancel request after it started executing and has not yet completed execution
    STATUS_RECALLING,      // = 7   # The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled
    STATUS_RECALLED,       // = 8   # The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
    STATUS_LOST            // = 9   # An action client can determine that a goal is LOST. This should not be sent over the wire by an action server
};

#endif

