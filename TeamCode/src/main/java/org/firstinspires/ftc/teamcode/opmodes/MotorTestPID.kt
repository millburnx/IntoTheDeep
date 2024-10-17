package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.common.subsystems.ArmPID
import org.firstinspires.ftc.teamcode.common.subsystems.MotorTestPIDConfig
import org.firstinspires.ftc.teamcode.common.subsystems.PID

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
    lateinit var armPID: ArmPID
    var target: Double = 40.0
    var pid = PID()

    override fun initialize() {
        motor = hardwareMap.get(DcMotor::class.java, "slides")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        armPID =
            ArmPID(hardwareMap)
    }

    override fun run() {
        armPID.target = 160
        armPID.run()

        val f = 0
        motor.power = pid.calc(MotorTestPIDConfig.target, motor.currentPosition.toInt()) + f

        val dash = FtcDashboard.getInstance().telemetry
        dash.addData("power", motor.power)
        dash.addData("target", MotorTestPIDConfig.target)
        dash.addData("position", motor.currentPosition)
        dash.update()
    }
}