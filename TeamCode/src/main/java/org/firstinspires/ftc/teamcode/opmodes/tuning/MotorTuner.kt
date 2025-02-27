package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.teamcode.common.utils.init

@TeleOp(name = "Motor Tuner", group = "Tuning")
@Config
class MotorTuner : CommandOpMode() {
    val motor by lazy { (hardwareMap[name] as DcMotorEx).apply { init(reverse) } }
    val multiTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun run() {
        motor.direction = if (reverse) Direction.REVERSE else Direction.FORWARD
        motor.power = power
        multiTelemetry.addData("motor", motor.currentPosition)
        multiTelemetry.update()
    }

    override fun initialize() {
    }

    companion object {
        @JvmField
        var power: Double = 0.0

        // IDK how to do port based without weird calcified stuff, I'll look into it later
        @JvmField
        var name: String = "servo"

        @JvmField
        var reverse: Boolean = false
    }
}
