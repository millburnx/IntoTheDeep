package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Config
@TeleOp(name = "Motor Test", group = "tuning")
class MotorTest : CommandOpMode() {
    val leftRotate: DcMotorEx by lazy {
        hardwareMap["leftRotate"] as DcMotorEx
    }

    val rightRotate: DcMotorEx by lazy {
        hardwareMap["rightRotate"] as DcMotorEx
    }

    fun resetEncoders() {
        leftRotate.direction = DcMotorSimple.Direction.FORWARD
        leftRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        rightRotate.direction = DcMotorSimple.Direction.REVERSE
        rightRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun initialize() {
        resetEncoders()
    }

    override fun run() {
        super.run()
        leftRotate.power = -power
        rightRotate.power = -power
    }

    companion object {
        @JvmField
        var power: Double = 0.0
    }
}