package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive.Companion.cacheThreshold
import org.firstinspires.ftc.teamcode.common.utils.CachedMotor
import org.firstinspires.ftc.teamcode.opmodes.tuning.DriveTuner.Companion.breakMotors

@TeleOp(name = "Drive Motor Tuner", group = "Tuning")
class DriveMotorTuner : CommandOpMode() {
    val frontLeft = CachedMotor(hardwareMap, "frontLeft", cacheThreshold, breakMotors)
    val frontRight = CachedMotor(hardwareMap, "frontRight", cacheThreshold, breakMotors, false)
    val backLeft = CachedMotor(hardwareMap, "backLeft", cacheThreshold, breakMotors)
    val backRight = CachedMotor(hardwareMap, "backRight", cacheThreshold, breakMotors, false)

    override fun initialize() {
    }

    override fun run() {
        frontLeft.power = if (gamepad1.triangle) 1.0 else 0.0
        frontRight.power = if (gamepad1.circle) 1.0 else 0.0
        backLeft.power = if (gamepad1.square) 1.0 else 0.0
        backRight.power = if (gamepad1.cross) 1.0 else 0.0

        telemetry.update()
    }
}
