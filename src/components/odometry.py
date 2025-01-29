from phoenix6.hardware import Pigeon2
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import Field2d, SmartDashboard, Timer
from wpimath.geometry import Pose2d, Translation2d

from components.swerve_drive import SwerveDrive
from magicbot import feedback, will_reset_to
from lemonlib.vision import LemonCamera
from lemonlib.preference import SmartProfile


class Odometry:
    camera: LemonCamera
    field_layout: AprilTagFieldLayout
    swerve_drive: SwerveDrive
    theta_profile: SmartProfile
    pigeon: Pigeon2

    request_face_tag = will_reset_to(False)

    def setup(self):
        self.estimated_field = Field2d()
        self.tag_object = self.estimated_field.getObject("tag")
        SmartDashboard.putData("Estimated Field", self.estimated_field)

    def on_enable(self):
        self.theta_controller = self.theta_profile.create_pid_controller("theta")

    def get_estimated_pose(self) -> None | Pose2d:
        if not self.camera.has_targets():
            return
        rt = self.camera.get_pose().translation()
        theta = self.pigeon.getRotation2d()
        rt = rt.rotateBy(theta)
        tag_pose = self.field_layout.getTagPose(self.camera.get_best_id())
        ot = Translation2d(tag_pose.x, tag_pose.y)
        return Pose2d(ot - rt, theta)

    def face_tag(self):
        self.request_face_tag = True

    def execute(self):
        self.camera.update()
        if self.camera.has_targets():
            self.swerve_drive.add_vision_measurement(
                self.get_estimated_pose(), Timer.getFPGATimestamp()
            )
            self.tag_object.setPose(
                self.field_layout.getTagPose(self.camera.get_best_id()).toPose2d()
            )
            """NOTE: all the theta_controller code should eventually
            be moved into a distinct drive_controller component"""
            tag_pose = self.camera.get_pose()
            tag_angle = tag_pose.translation().angle().degrees()
            robot_angle = self.get_estimated_pose().rotation().degrees()
            output = self.theta_controller.calculate(
                robot_angle, tag_angle + robot_angle
            )
            if self.request_face_tag:
                self.swerve_drive.set_rotationX(output)
        else:
            self.tag_object.setPose(Pose2d())
        self.estimated_field.setRobotPose(self.swerve_drive.get_estimated_pose())

    @feedback
    def get_best_x(self):
        if not self.camera.has_targets():
            return 0
        return self.camera.get_pose().translation().x

    @feedback
    def get_best_y(self):
        if not self.camera.has_targets():
            return 0
        return self.camera.get_pose().translation().y
