package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos

@Config
class ArmPID(hardwareMap: HardwareMap) {
    var leftRotate: DcMotorEx

    @JvmField
    var rightRotate: DcMotorEx

    private val controller: PIDController
    var target: Int = 0

    init {
        controller = PIDController(p, i, d)
        leftRotate = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "leftRotate")
        leftRotate.direction = DcMotorSimple.Direction.FORWARD
        leftRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        rightRotate = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "rightRotate")
        rightRotate.direction = DcMotorSimple.Direction.REVERSE
        rightRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun run() {
        controller.setPID(p, i, d)
        val pos = rightRotate.currentPosition
        val pid = controller.calculate(pos.toDouble(), target.toDouble())
        val ff = cos(Math.toRadians(target / ticks_in_degree)) * f

        val power = pid + ff

        leftRotate.power = -power
        rightRotate.power = -power
    }

    companion object {
        var p: Double = 0.0115
        var i: Double = 0.0
        var d: Double = 0.0
        var f: Double = 0.25

        var floor: Int = 10
        var up: Int = 140

        var ticks_in_degree: Double = 160.0 / 90.0
    }
}