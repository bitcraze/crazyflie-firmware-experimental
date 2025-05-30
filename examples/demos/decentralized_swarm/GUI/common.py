MAX_COPTERS = 9


class State:
    STATE_IDLE = 0
    STATE_WAIT_FOR_POSITION_LOCK = 1

    STATE_WAIT_FOR_TAKE_OFF = 2
    STATE_QUEUED_FOR_TAKE_OFF = 3
    STATE_PREPARING_FOR_TAKE_OFF = 4
    STATE_TAKING_OFF = 5
    STATE_HOVERING = 6
    STATE_GOING_TO_TRAJECTORY_START = 7
    STATE_EXECUTING_TRAJECTORY = 8
    STATE_GOING_TO_RANDOM_POINT = 9
    STATE_PREPARING_FOR_LAND = 10
    STATE_GOING_TO_PAD = 11

    STATE_WAITING_AT_PAD = 12
    STATE_LANDING = 13
    STATE_CHECK_CHARGING = 14
    STATE_REPOSITION_ON_PAD = 15
    STATE_CRASHED = 16
    STATE_SNIFFING = 17
    STATE_UNKNOWN = 255


state_dict = {
    State.STATE_IDLE: ["Idle", "purple"],
    State.STATE_WAIT_FOR_POSITION_LOCK: ["POS LOCK", "purple"],
    State.STATE_WAIT_FOR_TAKE_OFF: ["WAIT TAKE OFF", "purple"],
    State.STATE_QUEUED_FOR_TAKE_OFF: ["QUEUED FOR TAKE OFF", "purple"],
    State.STATE_PREPARING_FOR_TAKE_OFF: ["PREP TAKE OFF", "yellow"],
    State.STATE_TAKING_OFF: ["TAKING OFF", "green"],
    State.STATE_HOVERING: ["HOVERING", "green"],
    State.STATE_GOING_TO_TRAJECTORY_START: ["TRAJ START", "green"],
    State.STATE_EXECUTING_TRAJECTORY: ["EXEC TRAJ", "green"],
    State.STATE_GOING_TO_RANDOM_POINT: ["FLYING", "green"],
    State.STATE_PREPARING_FOR_LAND: ["PREP FOR LAND", "yellow"],
    State.STATE_GOING_TO_PAD: ["GOING TO PAD", "yellow"],
    State.STATE_WAITING_AT_PAD: ["WAITING AT PAD", "orange"],
    State.STATE_LANDING: ["LANDING", "orange"],
    State.STATE_CHECK_CHARGING: ["CHECK CHARGING", "purple"],
    State.STATE_REPOSITION_ON_PAD: ["REPOSITION ON PAD", "yellow"],
    State.STATE_CRASHED: ["CRASHED", "red"],
    State.STATE_SNIFFING: ["SNIFFING", "blue"],
}
