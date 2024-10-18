package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Limelight
import org.firstinspires.ftc.teamcode.common.utils.PoseColor
import org.firstinspires.ftc.teamcode.common.utils.Telemetry

@TeleOp(name = "Mega Tag")
class LimelightMT() : CommandOpMode() {
    val limelight: Limelight = Limelight(hardwareMap, telemetry)
    val drive: Drive = Drive(hardwareMap, -1.0)
    val telem: Telemetry = Telemetry()
    val dashTel = FtcDashboard.getInstance().telemetry

    val MTOIN = 39.27

    override fun initialize() {}

    override fun run() {
        val power = -gamepad1.left_stick_y.toDouble()
        val strafe = gamepad1.left_stick_x * 1.1
        val turn = gamepad1.right_stick_x.toDouble()
        drive.robotCentric(power, strafe, turn)
        drive.updatePos()

        val poses = mutableListOf<PoseColor>()
        poses.add(PoseColor(drive.getPos(), "#00ff00"))

        val orientation = drive.imu.robotYawPitchRollAngles
        val yaw = orientation.getYaw(AngleUnit.DEGREES)
        limelight.updateRobotOrientation(yaw)

        val results = limelight.getResults()
        if (results != null && results.isValid) {
            val mt1Pose = results.botpose
            val mt2Pose = results.botpose_MT2

            val mt1Pos = Pose2d(
                -mt1Pose.position.y * MTOIN,
                mt1Pose.position.x * MTOIN,
                Rotation2d(Math.toRadians(mt1Pose.orientation.yaw))
            )
            val mt2Pos = Pose2d(
                -mt2Pose.position.y * MTOIN,
                mt2Pose.position.x * MTOIN,
                Rotation2d(Math.toRadians(mt2Pose.orientation.yaw))
            )
            poses.add(PoseColor(mt1Pos, "#ff0000"))
            poses.add(PoseColor(mt2Pos, "#00ff00"))

            dashTel.addData("tx", results.tx * MTOIN)
            dashTel.addData("ty", results.ty * MTOIN)
        }

        telem.drawField(poses, FtcDashboard.getInstance())
    }
}