# from wpilib import Field2d, SmartDashboard, Timer
# from wpimath.geometry import Transform3d, Pose2d
# from photonlibpy.photonCamera import PhotonCamera
# from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
# from robotpy_apriltag import AprilTagFieldLayout

# from lemonlib.vision import LemonCamera

# from components.swerve_drive import SwerveDrive


# class Odometry:
#     camera_front: LemonCamera
#     camera_back: LemonCamera
#     robot_to_camera_front: Transform3d
#     robot_to_camera_back: Transform3d
#     field_layout: AprilTagFieldLayout
#     swerve_drive: SwerveDrive
#     estimated_field: Field2d

#     def setup(self):
#         self.camera_pose_estimator_front = PhotonPoseEstimator(
#             self.field_layout,
#             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
#             self.camera_front,
#             self.robot_to_camera_front,
#         )
#         self.camera_pose_estimator_back = PhotonPoseEstimator(
#             self.field_layout,
#             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
#             self.camera_back,
#             self.robot_to_camera_back,
#         )

#         SmartDashboard.putData("Estimated Field", self.estimated_field)

#     def execute(self):
#         # may need to tweak timestamp to match system time
#         self.camera_front.update()
#         camera_estimator_result_front = self.camera_pose_estimator_front.update()
#         camera_estimator_result_back = self.camera_pose_estimator_back.update()
#         if camera_estimator_result_front is not None:
#             self.swerve_drive.add_vision_measurement(
#                 camera_estimator_result_front.estimatedPose.toPose2d(),
#                 self.camera_front.getLatestResult().getTimestampSeconds(),
#             )
#         if camera_estimator_result_back is not None:
#             self.swerve_drive.add_vision_measurement(
#                 camera_estimator_result_back.estimatedPose.toPose2d(),
#                 self.camera_back.getLatestResult().getTimestampSeconds(),
#             )
#         self.estimated_field.setRobotPose(self.swerve_drive.get_estimated_pose())
#         if self.swerve_drive.has_desired_pose:
#             self.estimated_field.getObject("desired").setPose(
#                 self.swerve_drive.desired_pose
#             )
