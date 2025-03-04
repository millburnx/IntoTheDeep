package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.utils.AxonCR
import org.firstinspires.ftc.teamcode.common.utils.DeltaTime

@TeleOp(name = "Axon Analog Tuner", group = "Tuning")
@Config
class AxonAnalogTuner : CommandOpMode() {
    val servo by lazy { AxonCR(hardwareMap, servoName, name) }
    val deltaTime = DeltaTime()
    val multiTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun run() {
        super.run()

        servo.power = servoPower
        multiTelemetry.addData("raw position", servo.rawPosition)
        multiTelemetry.addData("corrected position", servo.position)
        multiTelemetry.addData("Loop Hertz", 1.0 / deltaTime.deltaTime)
        multiTelemetry.update()
    }

    override fun initialize() {}

    companion object {
        // IDK how to do port based without weird calcified stuff, I'll look into it later
        @JvmField
        var name: String = "analog1"

        @JvmField
        var servoName: String = "diffyRight"

        @JvmField
        var servoPower: Double = 0.0
    }
}
