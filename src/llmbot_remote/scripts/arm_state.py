from enum import Enum

class ArmState(str, Enum):
    PLANNING = "Planning"
    PLANNING_SUCCESS = "PlanningSuccess"
    EXECUTING = "Executing"
    EXECUTION_COMPLETED = "ExecutionCompleted"
    EXECUTION_FAILED = "ExecutionFailed"
    PLANNING_FAILED = "PlanningFailed"
    PREEMPTED = "Preempted"
