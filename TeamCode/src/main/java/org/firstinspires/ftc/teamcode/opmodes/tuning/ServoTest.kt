package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@Config
object ServoTestConfig {
    @JvmField
    var position = 0.0
}

@TeleOp(name = "Servo Test", group = "Tuning")
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