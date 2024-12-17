package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo.Direction
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.common.utils.init

@TeleOp(name = "Servo Tuner")
@Config
class ServoTuner : CommandOpMode() {
    val servo by lazy { (hardwareMap[name] as ServoImplEx).apply { init(reverse) } }
    val servo2 by lazy { (hardwareMap[name2] as ServoImplEx).apply { init(reverse2) } }
    val multiTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun run() {
        if (axon) {
            servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
            servo2.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        }
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
        var axon: Boolean = true

        @JvmField
        var name2: String = "servo2"

        @JvmField
        var reverse2: Boolean = true
    }
}