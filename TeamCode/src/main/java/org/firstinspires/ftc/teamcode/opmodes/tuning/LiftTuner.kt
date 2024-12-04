package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

@TeleOp(name = "Lift Tuner")
@Config
class LiftTuner : CommandOpMode() {
    val lift by lazy { hardwareMap["lift"] as DcMotorEx }
    val pid = PIDController(kP, kI, kD)
    val multiTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun initialize() {
        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS
    }

    override fun run() {
        pid.setPID(kP, kI, kD)
        val power = pid.calculate(lift.currentPosition.toDouble(), target.toDouble()) + kF
        lift.power = power + kF

        multiTelemetry.addLine("Lift Power: $power")
        multiTelemetry.addLine("Lift Position: ${lift.currentPosition}")
        multiTelemetry.addLine("Target: $target")
        multiTelemetry.update()
    }

    companion object {
        @JvmField
        var target: Int = 0

        @JvmField
        var kP: Double = 0.03

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kF: Double = 0.01
    }
}