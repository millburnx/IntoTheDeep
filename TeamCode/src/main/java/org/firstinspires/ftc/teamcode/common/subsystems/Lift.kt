package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

@Config
class Lift(hardwareMap: HardwareMap, var armAngle: () -> Double = { 90.0 }) : SubsystemBase() {
    val lift: DcMotorEx by lazy {
        hardwareMap["slides"] as DcMotorEx
    }

    fun resetEncoders() {
        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lift.direction = DcMotorSimple.Direction.REVERSE
    }

    val controller: PIDController = PIDController(p, i, d)
    val position: Int
        get() = lift.currentPosition
    var target: Double = 50.0

    fun ticksToInches(ticks: Double): Double {
        return 0.0102 * ticks + 17.1
    }

    fun inchesToTicks(inches: Double): Double {
        return (inches - 17.1) / 0.0102
    }

    val maxLiftPosition: Vec2d
        get() = Vec2d(
            cos(armAngle()),
            sin(armAngle())
        ) * if (armAngle() == 0.0) Double.POSITIVE_INFINITY else (extensionLimit / cos(armAngle()))

    val maxLiftDistance: Double
        get() = Vec2d(0, 0).distanceTo(maxLiftPosition)

    init {
        lift.direction = DcMotorSimple.Direction.FORWARD
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        resetEncoders()
    }

    override fun periodic() {
        run()
    }

    fun run() {
        controller.setPID(p, i, d)
        val targetInches = ticksToInches(target)
        val clampedTarget = if (!useExtensionLimit) targetInches else min(maxLiftDistance, targetInches)
        val clampedTargetTicks = inchesToTicks(clampedTarget)
        val pid = controller.calculate(position.toDouble(), clampedTargetTicks.toDouble())
        val ff = sin(armAngle()) * f

        val power = pid + ff
        lift.power = power
    }

    companion object {
        @JvmField
        var p: Double = 0.002

        @JvmField
        var i: Double = 0.15

        @JvmField
        var d: Double = 0.0

        @JvmField
        var f: Double = 0.065

        @JvmField
        var base = 10;

        @JvmField
        var pickup = 500;

        @JvmField
        var lowBasket = 1150;

        @JvmField
        var highBasket = 1150;

        @JvmField
        var useExtensionLimit = false // enable after tuned

        @JvmField
        var extensionLimit = 40.0 // need to measure
    }
}