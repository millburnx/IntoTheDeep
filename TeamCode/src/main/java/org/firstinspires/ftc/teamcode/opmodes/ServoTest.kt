package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction

@TeleOp(name = "Servo Test")
class ServoTest : CommandOpMode() {
    val servo: CRServo by lazy {
        hardwareMap["Intake"] as CRServo
    }

    override fun initialize() {
        servo.direction = Direction.FORWARD
    }

    override fun run() {
        super.run()

        servo.power = power
    }

    companion object {
        @JvmField
        val power = 0.0
    }
}