package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
object LiftPIDConfig {
    @JvmField
    val kP = 0.02

    @JvmField
    val kI = 0.0

    @JvmField
    val kD = 0.0

    @JvmField
    val kF = 0.0

    @JvmField
    var target = 10
}

class LiftPID(hardwareMap: HardwareMap) : SubsystemBase() {
    val lift: DcMotorEx by lazy {
        val lift = hardwareMap["slides"] as DcMotorEx
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift.direction = DcMotorSimple.Direction.FORWARD
        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        return@lazy lift
    }
    val pid = PIDController(LiftPIDConfig.kP, LiftPIDConfig.kI, LiftPIDConfig.kD)

    fun run() {
        pid.setPID(LiftPIDConfig.kP, LiftPIDConfig.kI, LiftPIDConfig.kD)

        val pos = lift.currentPosition
        val power = pid.calculate(pos.toDouble(), LiftPIDConfig.target.toDouble())
        val finalPower = power + LiftPIDConfig.kF
        lift.power = finalPower
    }

    fun setTarget(targetPos: Int) {
        LiftPIDConfig.target = targetPos
    }
}