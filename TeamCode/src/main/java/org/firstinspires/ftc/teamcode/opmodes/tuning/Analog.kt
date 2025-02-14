package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.common.utils.DeltaTime

@TeleOp(name = "Analog Tuner")
@Config
class Analog : CommandOpMode() {
    val input by lazy { (hardwareMap[name] as AnalogInput) }
    val deltaTime = DeltaTime()
    val multiTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun run() {
        super.run()
        multiTelemetry.addData("voltage", input.voltage)
        multiTelemetry.addData("max voltage", input.maxVoltage)
        multiTelemetry.addData("Loop Hertz", 1.0 / deltaTime.deltaTime)
        multiTelemetry.update()
    }

    override fun initialize() {}

    companion object {
        // IDK how to do port based without weird calcified stuff, I'll look into it later
        @JvmField
        var name: String = "intakeClaw"
    }
}
