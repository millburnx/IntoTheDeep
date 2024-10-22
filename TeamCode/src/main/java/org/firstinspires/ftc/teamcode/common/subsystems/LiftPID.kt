package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos

@Config
class LiftPID(hardwareMap: HardwareMap) {
    @JvmField
    var lift: DcMotorEx

    private val controller: PIDController
    var target: Int = 10

    init {
        controller = PIDController(p, i, d)
        lift = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "slides")
        lift.direction = DcMotorSimple.Direction.REVERSE
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun run() {
        controller.setPID(p, i, d)
        val pos = lift.currentPosition
        val pid = controller.calculate(pos.toDouble(), target.toDouble())
        val ff = cos(Math.toRadians(target / ticks_in_degree)) * f

        val power = pid + ff

        lift.power = power
    }

    @Config
    companion object LiftPIDConfig {
        @JvmField
        var p: Double = 0.005

        @JvmField
        var i: Double = 0.0

        @JvmField
        var d: Double = 0.0

        @JvmField
        var f: Double = 0.065

        @JvmField
        var ticks_in_degree: Double = 8192.0 / 360.0

        @JvmField
        var base = 100;

        @JvmField
        var pickup = 500;

        @JvmField
        var first = 1200;
    }
}