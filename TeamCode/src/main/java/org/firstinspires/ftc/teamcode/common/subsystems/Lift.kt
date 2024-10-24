package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos

@Config
class Lift(hardwareMap: HardwareMap) : SubsystemBase() {
    val lift: DcMotorEx by lazy {
        hardwareMap["slides"] as DcMotorEx
    }

    val controller: PIDController = PIDController(p, i, d)
    val position: Int
        get() = lift.currentPosition
    var target: Int = 10

    init {
        lift.direction = DcMotorSimple.Direction.REVERSE
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun periodic() {
        run()
    }

    fun run() {
        controller.setPID(p, i, d)
        val pid = controller.calculate(position.toDouble(), target.toDouble())
        val ff = cos(Math.toRadians(target / ticks_in_degree)) * f

        val power = pid + ff
        lift.power = power
    }

    companion object {
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
        var lowBasket = 1150;

        @JvmField
        var highBasket = 1150;
    }
}