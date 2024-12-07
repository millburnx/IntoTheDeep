package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.utils.init

@TeleOp(name = "Motor PIDF Tuner")
@Config
class MotorPIDFTuner : CommandOpMode() {
    val motor by lazy { hardwareMap[name] as DcMotorEx }
    val pid = PIDController(kP, kI, kD)
    val multiTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun initialize() {
        motor.init()
    }

    override fun run() {
        pid.setPID(kP, kI, kD)
        val power = pid.calculate(motor.currentPosition.toDouble(), target.toDouble()) + kF
        motor.power = power + kF

        multiTelemetry.addLine("Motor Power: $power")
        multiTelemetry.addLine("Motor Position: ${motor.currentPosition}")
        multiTelemetry.addLine("Target: $target")
        multiTelemetry.update()
    }

    companion object {
        @JvmField
        var name: String = "motor"

        @JvmField
        var target: Int = 0

        @JvmField
        var kP: Double = 0.0

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kF: Double = 0.00
    }
}