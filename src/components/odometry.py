from wpilib import Field2d, SmartDashboard, Timer
from wpimath.geometry import Transform3d, Pose2d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagFieldLayout

from components.swerve_drive import SwerveDrive


class Odometry:
    camera: PhotonCamera
    robot_to_camera: Transform3d
    field_layout: AprilTagFieldLayout
    swerve_drive: SwerveDrive
    estimated_field: Field2d

    def setup(self):
        self.camera_pose_estimator = PhotonPoseEstimator(
            self.field_layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera,
            self.robot_to_camera,
        )
        
        SmartDashboard.putData("Estimated Field", self.estimated_field)

    def execute(self):
        # may need to tweak timestamp to match system time
        camera_estimator_result = self.camera_pose_estimator.update()
        if camera_estimator_result is not None:
            self.swerve_drive.add_vision_measurement(
                camera_estimator_result.estimatedPose.toPose2d(),
                self.camera.getLatestResult().getTimestampSeconds(),
            )
        self.estimated_field.setRobotPose(self.swerve_drive.get_estimated_pose())
