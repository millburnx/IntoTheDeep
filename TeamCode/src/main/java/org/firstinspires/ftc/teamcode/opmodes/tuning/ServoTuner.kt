package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import org.firstinspires.ftc.teamcode.common.utils.init

@TeleOp(name = "Servo Tuner")
@Config
class ServoTuner : CommandOpMode() {
    val servo by lazy { (hardwareMap[name] as Servo).apply { init(reverse) } }
    val servo2 by lazy { (hardwareMap[name2] as Servo).apply { init(reverse2) } }
    val multiTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun run() {
        servo.direction = if (reverse) Direction.REVERSE else Direction.FORWARD
        servo2.direction = if (reverse2) Direction.REVERSE else Direction.FORWARD
        servo.position = position
        servo2.position = position
        multiTelemetry.update()
    }

    override fun initialize() {
    }

    companion object {
        @JvmField
        var position: Double = 0.0

        // IDK how to do port based without weird calcified stuff, I'll look into it later
        @JvmField
        var name: String = "servo"

        @JvmField
        var reverse: Boolean = false

        @JvmField
        var name2: String = "servo2"

        @JvmField
        var reverse2: Boolean = false
    }
}