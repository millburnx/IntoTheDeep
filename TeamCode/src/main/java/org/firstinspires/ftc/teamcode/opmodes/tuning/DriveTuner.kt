package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.arcrobotics.ftclib.command.CommandOpMode
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive.Companion.cacheThreshold
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive.Companion.strafeMultiplier
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedMotor
import kotlin.math.absoluteValue
import kotlin.math.max

@TeleOp(name = "Drive Tuner", group = "Tuning")
class DriveTuner : CommandOpMode() {
    val frontLeft = CachedMotor(hardwareMap, "frontLeft", cacheThreshold, breakMotors)
    val frontRight = CachedMotor(hardwareMap, "frontRight", cacheThreshold, breakMotors, false)
    val backLeft = CachedMotor(hardwareMap, "backLeft", cacheThreshold, breakMotors)
    val backRight = CachedMotor(hardwareMap, "backRight", cacheThreshold, breakMotors, false)

    override fun initialize() {
    }

    override fun run() {
        val heading = 0.0
        val x = gamepad1.left_stick_y.toDouble()
        val y = -gamepad1.left_stick_x.toDouble()
        val rotate = -gamepad1.right_stick_x.toDouble()

        val relativeVector = Vec2d(x, y).rotate(-heading) * Vec2d(1.0, strafeMultiplier)

        val forward = relativeVector.x
        val strafe = relativeVector.y

        val denominator = max(forward.absoluteValue + strafe.absoluteValue + rotate.absoluteValue, 1.0)
        frontLeft.power = (forward + strafe + rotate) / denominator
        backLeft.power = (forward - strafe + rotate) / denominator
        frontRight.power = (forward - strafe - rotate) / denominator
        backRight.power = (forward + strafe - rotate) / denominator

        telemetry.addData("lx", gamepad1.left_stick_x)
        telemetry.addData("ly", gamepad1.left_stick_y)
        telemetry.addData("rx", gamepad1.right_stick_x)
        telemetry.addData("ry", gamepad1.right_stick_y)
        telemetry.addData("relative x", relativeVector.x)
        telemetry.addData("relative y", relativeVector.y)
        telemetry.update()
    }

    companion object {
        @JvmField
        var breakMotors: Boolean = false
    }
}
