package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.common.subsystems.ArmPID
import org.firstinspires.ftc.teamcode.common.subsystems.MotorTestPIDConfig
import org.firstinspires.ftc.teamcode.common.subsystems.PID

@TeleOp(name = "Motor Test PID")
class MotorTestPid() : CommandOpMode() {
    val motor: DcMotor by lazy {
        val motor = hardwareMap["slides"] as DcMotor
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        return@lazy motor
    }
    val armPID: ArmPID = ArmPID(hardwareMap)
    val pid = PID()

    override fun initialize() {}

    override fun run() {
        armPID.setTarget(160)
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