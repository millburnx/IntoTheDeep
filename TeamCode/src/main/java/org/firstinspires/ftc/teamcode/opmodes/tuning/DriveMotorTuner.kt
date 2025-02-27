package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.utils.init

@TeleOp(name = "Drive Motor Tuner", group = "Tuning")
class DriveMotorTuner : CommandOpMode() {
    val frontLeft: DcMotorEx by lazy { (hardwareMap["frontLeft"] as DcMotorEx).apply { init(false) } }
    val frontRight: DcMotorEx by lazy { (hardwareMap["frontRight"] as DcMotorEx).apply { init() } }
    val backLeft: DcMotorEx by lazy { (hardwareMap["backLeft"] as DcMotorEx).apply { init(false) } }
    val backRight: DcMotorEx by lazy { (hardwareMap["backRight"] as DcMotorEx).apply { init() } }

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
