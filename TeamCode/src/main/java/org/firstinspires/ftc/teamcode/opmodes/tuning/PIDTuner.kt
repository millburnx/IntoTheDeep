package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings
import org.firstinspires.ftc.teamcode.common.commands.drive.PickupPIDSettings
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonRobot

@TeleOp(name = "PID Tuner", group = "Tuning")
@Config
class PIDTuner : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    override fun exec() {
        robot.drive.pidManager.isOn = true
        robot.drive.pidManager.target = Pose2d(x, y, h)
        robot.telemetry.addData("at target", robot.drive.pidManager.atTarget())
        if (isPickup) {
            robot.drive.pidManager.kP = PickupPIDSettings.kP
            robot.drive.pidManager.kI = PickupPIDSettings.kI
            robot.drive.pidManager.kD = PickupPIDSettings.kD
            robot.drive.pidManager.kPHeading = PickupPIDSettings.kPHeading
            robot.drive.pidManager.kIHeading = PickupPIDSettings.kIHeading
            robot.drive.pidManager.kDHeading = PickupPIDSettings.kDHeading
            robot.drive.pidManager.tolerance = Pose2d(PickupPIDSettings.tolerance, PickupPIDSettings.headingTolerance)
        } else {
            robot.drive.pidManager.kP = PIDSettings.kP
            robot.drive.pidManager.kI = PIDSettings.kI
            robot.drive.pidManager.kD = PIDSettings.kD
            robot.drive.pidManager.kPHeading = PIDSettings.kPHeading
            robot.drive.pidManager.kIHeading = PIDSettings.kIHeading
            robot.drive.pidManager.kDHeading = PIDSettings.kDHeading
            robot.drive.pidManager.tolerance = Pose2d(PIDSettings.tolerance, PIDSettings.headingTolerance)
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
