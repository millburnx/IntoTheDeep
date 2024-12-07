package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "Servo Tuner")
@Config
class ServoTuner : CommandOpMode() {
    val servo by lazy { hardwareMap[name] as Servo }
    val multiTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun run() {
        servo.position = position
        multiTelemetry.update()
    }

    override fun initialize() {
    }

    companion object {
        @JvmField
        var position: Double = 0.0

        // idk how to do port based without weird calcified stuff, i'll look into it later
        @JvmField
        var name: String = "servo"
    }
}