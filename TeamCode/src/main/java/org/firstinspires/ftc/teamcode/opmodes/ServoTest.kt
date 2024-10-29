package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "Servo Test")
class ServoTest : CommandOpMode() {
    val servo: Servo by lazy {
        hardwareMap["Intake"] as Servo
    }

    override fun initialize() {
    }

    override fun run() {
        super.run()
        servo.position = ServoTestConfig.position
    }
}