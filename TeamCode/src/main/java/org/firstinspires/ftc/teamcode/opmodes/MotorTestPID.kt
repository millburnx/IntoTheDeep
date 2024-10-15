package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.common.subsystems.MotorTestPIDConfig
import org.firstinspires.ftc.teamcode.common.subsystems.PID
import kotlin.math.cos

@Config
object ServoTestConfig {
    @JvmField
    var power = 0.0
}

@TeleOp(name = "Motor Test PID")
class MotorTestPid() : CommandOpMode() {
    val ticksToDegrees = -160.0 / 90.0
    val startingAngle = 0.0 * ticksToDegrees

    lateinit var motor: DcMotor;
    var target: Double = 40.0
    var pid = PID()

    override fun initialize() {
        motor = hardwareMap.get(DcMotor::class.java, "rotate")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun run() {
        val currentAngle = motor.currentPosition.toDouble() * ticksToDegrees + startingAngle
        val f = MotorTestPIDConfig.kF * cos(Math.toRadians(target))
        motor.power = pid.calc(MotorTestPIDConfig.target, currentAngle) + f

        val dash = FtcDashboard.getInstance().telemetry
        dash.addData("power", motor.power)
        dash.addData("target", target)
        dash.addData("position", motor.currentPosition)
        dash.update()
    }
}