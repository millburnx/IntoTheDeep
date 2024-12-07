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
    val servo by lazy { hardwareMap["servo"] as Servo }
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

        // .575(extended) | 0.975(base) for linkage
    }
}