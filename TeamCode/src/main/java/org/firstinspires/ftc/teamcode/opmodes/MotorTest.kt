package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Config
object MOTORPOWER {
    @JvmField
    var power = 0.0
}

@TeleOp(name = "Motor Test")
class MotorTest() : CommandOpMode() {
    lateinit var motor: DcMotor;
    lateinit var motor2: DcMotor;

    override fun initialize() {
        motor = hardwareMap.get(DcMotor::class.java, "rotate")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.FORWARD

        motor2 = hardwareMap.get(DcMotor::class.java, "rotate2")
        motor2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor2.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun run() {
        motor.power = MOTORPOWER.power
        motor2.power = MOTORPOWER.power
    }
}