package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.utils.DeltaTime
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedServo
import org.firstinspires.ftc.teamcode.common.utils.hardware.ServoLimiter

@TeleOp(name = "Servo Tuner", group = "Tuning")
@Config
class ServoTuner : CommandOpMode() {
    val servo by lazy { CachedServo(hardwareMap, name, isAxon = axon, isForward = reverse) }
    val servo2 by lazy { CachedServo(hardwareMap, name2, isAxon = axon, isForward = reverse2) }
    val deltaTime = DeltaTime()
    val servoLimiter = ServoLimiter(maxSpeed, { deltaTime.deltaTime }, position)
    val multiTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun run() {
        super.run()
        servoLimiter.maxSpeed = maxSpeed
        servoLimiter.update(position)
        if (maxSpeed == -1.0) {
            servo.position = position
            servo2.position = position
        } else {
            servo.position = servoLimiter.current
            servo2.position = servoLimiter.current
        }
        multiTelemetry.addData("position", servoLimiter.current)
        multiTelemetry.update()
    }

    override fun initialize() {}

    companion object {
        @JvmField
        var maxSpeed: Double = Double.MAX_VALUE // per second

        @JvmField
        var position: Double = 0.0

        // IDK how to do port based without weird calcified stuff, I'll look into it later
        @JvmField
        var name: String = "intakeClaw"

        @JvmField
        var reverse: Boolean = false

        @JvmField
        var axon: Boolean = true

        @JvmField
        var name2: String = "a"

        @JvmField
        var reverse2: Boolean = true
    }
}
