package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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
        robot.drive.pidManager.isSamplePickup = isPickup
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
