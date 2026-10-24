import unittest

from vgm_task_manager.model import Candidate, TaskCoordinator, TaskState


class TestTaskCoordinator(unittest.TestCase):
    def test_selects_highest_confidence_matching_target(self):
        coordinator = TaskCoordinator("workpiece", 0.7)
        chosen = coordinator.select(
            [
                Candidate("a", "workpiece", 0.8, 0.5),
                Candidate("b", "other", 0.99, 0.2),
                Candidate("c", "workpiece", 0.9, 0.7),
            ]
        )
        self.assertEqual(chosen.object_id, "c")
        self.assertIs(coordinator.state, TaskState.TARGET_SELECTED)

    def test_rejects_invalid_depth_and_low_confidence(self):
        coordinator = TaskCoordinator("workpiece", 0.8)
        self.assertIsNone(
            coordinator.select(
                [Candidate("a", "workpiece", 0.7, 1.0), Candidate("b", "workpiece", 0.9, 0.0)]
            )
        )

    def test_lifecycle_reaches_success_and_resets(self):
        coordinator = TaskCoordinator("workpiece")
        coordinator.select([Candidate("a", "workpiece", 0.9, 0.5)])
        coordinator.mark_dispatched()
        coordinator.mark_running()
        coordinator.complete(True)
        self.assertIs(coordinator.state, TaskState.SUCCEEDED)
        coordinator.reset()
        self.assertIs(coordinator.state, TaskState.IDLE)

