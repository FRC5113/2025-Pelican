from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d, Transform3d


class LemonCameraSim(LemonCamera):
    """Simulated version of a LemonCamera. This class functions exactly
    the same in code except for the following:
    1. Must be initialized with an `AprilTagFieldLayout` and an FOV
    2. `set_robot_pose()` must be called periodically to update the pose
    of the robot. This should not be taken from a pose estimator that
    uses vision updates, but rather a pose simulated in physics.py
    3. This simulation assumes that the camera is at the center of the
    robot looking directly forward, but the difference should be negligible
    """

    def __init__(
        self,
        field_layout: AprilTagFieldLayout,
        fov: float,
        camera_to_bot: Transform3d,
    ):
        """Args:
        field_layout (AprilTagFieldLayout): layout of the tags on the field, such as
            `AprilTagField.k2024Crescendo`
        fov (float): horizontal range of vision (degrees)
        """
        LemonCamera.__init__(self, "Sim", camera_to_bot)
        self.field_layout = field_layout
        self.fov = fov
        self.robot_pose = None
        self.tag_poses = {}
        self.tag_ambiguities = {}
        self.latency = 0
        self.camera_to_bot = camera_to_bot

    def set_robot_pose(self, pose: Pose2d):
        self.robot_pose = pose

    def update(self):
        if self.robot_pose is None:
            return
        self.tag_poses = {}
        self.tag_ambiguities = {}
        self.latency = 0.02
        for tag in self.field_layout.getTags():
            tag_pose = tag.pose.toPose2d()
            relative_pose = tag_pose.relativeTo(
                self.robot_pose.transformBy(self.camera_to_bot)
            )
            dist = relative_pose.translation().norm()
            # calculate estimated tag ambiguity based on distance and angle
            ambiguity = (
                -dist * dist / (tag_pose.rotation() - self.robot_pose.rotation()).cos()
            )
            # check that robot can "see" tag
            if (
                abs(relative_pose.translation().angle().degrees()) < self.fov / 2
                and ambiguity > 0
            ):
                self.tag_ambiguities[tag.ID] = ambiguity
                self.tag_poses[tag.ID] = relative_pose
