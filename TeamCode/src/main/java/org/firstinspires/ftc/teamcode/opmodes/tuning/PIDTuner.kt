package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings
import org.firstinspires.ftc.teamcode.common.commands.drive.PickupPIDSettings
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonRobot

@TeleOp(name = "PID Tuner")
@Config
class PIDTuner : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    override fun exec() {
        robot.pidManager.isOn = true
        robot.pidManager.target = Pose2d(Vec2d(x, y), h)
        robot.telemetry.addData("at target", robot.pidManager.atTarget())
        if (isPickup) {
            robot.pidManager.kP = PickupPIDSettings.kP
            robot.pidManager.kI = PickupPIDSettings.kI
            robot.pidManager.kD = PickupPIDSettings.kD
            robot.pidManager.kPHeading = PickupPIDSettings.kPHeading
            robot.pidManager.kIHeading = PickupPIDSettings.kIHeading
            robot.pidManager.kDHeading = PickupPIDSettings.kDHeading
            robot.pidManager.tolerance = Pose2d(PickupPIDSettings.tolerance, PickupPIDSettings.headingTolerance)
        } else {
            robot.pidManager.kP = PIDSettings.kP
            robot.pidManager.kI = PIDSettings.kI
            robot.pidManager.kD = PIDSettings.kD
            robot.pidManager.kPHeading = PIDSettings.kPHeading
            robot.pidManager.kIHeading = PIDSettings.kIHeading
            robot.pidManager.kDHeading = PIDSettings.kDHeading
            robot.pidManager.tolerance = Pose2d(PIDSettings.tolerance, PIDSettings.headingTolerance)
        }
    }

    companion object {
        @JvmField
        var x: Double = 0.0

        @JvmField
        var y: Double = 0.0

        @JvmField
        var h: Double = 0.0

        @JvmField
        var isPickup: Boolean = false
    }
}
