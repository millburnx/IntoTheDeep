package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.common.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.common.subsystems.Limelight
import org.firstinspires.ftc.teamcode.common.utils.PoseColor
import org.firstinspires.ftc.teamcode.common.utils.Telemetry

@TeleOp(name = "Mega Tag 2")
class LimelightMT2() : CommandOpMode() {
    lateinit var limelight: Limelight;
    lateinit var drive: DriveSubsystem
    lateinit var telem: Telemetry
    override fun initialize() {
        limelight = Limelight(hardwareMap, telemetry)
        drive = DriveSubsystem(hardwareMap, -1.0)
        telem = Telemetry()
    }

    override fun run() {
        require(::limelight.isInitialized)
        require(::drive.isInitialized)
        require(::telem.isInitialized)
        super.run()

        val power = -gamepad1.left_stick_y.toDouble();
        val strafe = gamepad1.left_stick_x * 1.1;
        val turn = gamepad1.right_stick_x.toDouble();
        drive.robotCentric(power, strafe, turn);
        drive.updatePos();

        val poses = mutableListOf<PoseColor>()
        val odom = drive.getPos()
        poses.add(PoseColor(odom, "#0000ff"))

        val orientation = drive.imu.robotYawPitchRollAngles
        val yaw = orientation.getYaw(AngleUnit.DEGREES)
        limelight.updateRobotOrientation(yaw)
        val result = limelight.getResults()
        if (result != null && result.isValid) {
            val pose = result.botpose_MT2
            val mtoin = 39.37
            FtcDashboard.getInstance().telemetry.addData("tx", result.tx * mtoin)
            FtcDashboard.getInstance().telemetry.addData("ty", result.ty * mtoin)
            FtcDashboard.getInstance().telemetry.update()
            poses.add(
                PoseColor(
                    Pose2d(
                        -pose.position.y * mtoin,
                        pose.position.x * mtoin,
                        Rotation2d(Math.toRadians(pose.orientation.yaw))
                    ), "#ff0000"
                )
            )
        }
        telem.drawField(poses, FtcDashboard.getInstance())
    }
}