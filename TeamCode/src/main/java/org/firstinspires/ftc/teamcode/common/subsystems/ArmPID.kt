package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos

@Config
object ArmPIDConfig {
    @JvmField
    var kP = 0.0115

    @JvmField
    var kI = 0.0

    @JvmField
    var kD = 0.0

    @JvmField
    var kF = 0.25

    @JvmField
    var floor = 10

    @JvmField
    var up = 140

    @JvmField
    var ticks_in_degree = 160.0 / 90.0

    @JvmField
    var target = 0
}

class ArmPID(hardwareMap: HardwareMap) : SubsystemBase() {
    val leftRotate: DcMotor by lazy {
        val leftRotate = hardwareMap["leftRotate"] as DcMotor
        leftRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftRotate.direction = DcMotorSimple.Direction.FORWARD
        leftRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        leftRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        return@lazy leftRotate

    }
    val rightRotate: DcMotor by lazy {
        val rightRotate = hardwareMap["rightRotate"] as DcMotor
        rightRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightRotate.direction = DcMotorSimple.Direction.REVERSE;
        rightRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        rightRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        return@lazy rightRotate
    }
    val pid = PIDController(ArmPIDConfig.kP, ArmPIDConfig.kI, ArmPIDConfig.kD)

    fun run() {
        pid.setPID(ArmPIDConfig.kP, ArmPIDConfig.kI, ArmPIDConfig.kD)

        val pos = rightRotate.currentPosition
        val power = pid.calculate(pos.toDouble(), ArmPIDConfig.target.toDouble())
        val currentAngle = ArmPIDConfig.target / ArmPIDConfig.ticks_in_degree
        val ff = cos(Math.toRadians(currentAngle)) * ArmPIDConfig.kF

        val finalPower = power + ff
        leftRotate.power = -finalPower
        rightRotate.power = -finalPower
    }

    fun setTarget(targetPos: Int) {
        ArmPIDConfig.target = targetPos
    }
}