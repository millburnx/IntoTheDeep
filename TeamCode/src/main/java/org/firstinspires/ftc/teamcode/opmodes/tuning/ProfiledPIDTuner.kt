package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.drive.ProfiledPIDManager
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonRobot

class ProfiledAutonRobot(
    opMode: OpMode,
) : AutonRobot(opMode) {
    override val drive = ProfiledPIDDrive(this)
}

class ProfiledPIDDrive(
    robot: Robot,
) : Drive(robot) {
    override val pidManager: ProfiledPIDManager = ProfiledPIDManager(robot)
}

@TeleOp(name = "Profiled PID Tuner", group = "Tuning")
@Config
class ProfiledPIDTuner : OpMode() {
    override val robot by lazy { ProfiledAutonRobot(this) }

    override fun exec() {
        robot.drive.pidManager.isOn = true

        val target = robot.drive.pidManager.target
        if (target.x != x || target.y != y || target.heading != h) {
            robot.drive.pidManager.target = Pose2d(x, y, h)
        }
        val pTarget = robot.drive.pidManager.profiledTarget

        robot.telemetry.addData("profiled target x", robot.drive.pidManager.profiledTarget.x)
        robot.telemetry.addData("profiled target y", robot.drive.pidManager.profiledTarget.y)
        robot.telemetry.addData("profiled target h", robot.drive.pidManager.profiledTarget.heading)
        robot.telemetry.addData("velocity t", robot.drive.pidManager.currentTransVelocity)
        robot.telemetry.addData("velocity h", robot.drive.pidManager.currentHeadingVelocity)
        robot.telemetry.addData("at target", robot.drive.pidManager.atTarget())

        val canvas = robot.telemetryManager.currentPacket.fieldOverlay()
        canvas.setStroke(targetColor)
        canvas.fillCircle(target.toRR().x, target.toRR().y, targetSize / 2)
        canvas.setStroke(pTargetColor)
        canvas.fillCircle(pTarget.toRR().x, pTarget.toRR().y, targetSize / 2)
    }

    companion object {
        @JvmField
        var x: Double = 0.0

        @JvmField
        var y: Double = 0.0

        @JvmField
        var h: Double = 0.0

        @JvmField
        var targetSize = 4.0

        @JvmField
        var targetColor = "#ff0000"

        @JvmField
        var pTargetColor = "#00ff00"
    }
}
