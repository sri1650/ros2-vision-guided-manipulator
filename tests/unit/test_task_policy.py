from vgm_task_manager.model import Candidate, TaskCoordinator, TaskState


def test_plan_task_policy_selects_then_completes():
    coordinator = TaskCoordinator("workpiece", 0.75)
    selected = coordinator.select([Candidate("part-01", "workpiece", 0.92, 0.48)])
    assert selected is not None
    coordinator.mark_dispatched()
    coordinator.complete(True)
    assert coordinator.state is TaskState.SUCCEEDED

