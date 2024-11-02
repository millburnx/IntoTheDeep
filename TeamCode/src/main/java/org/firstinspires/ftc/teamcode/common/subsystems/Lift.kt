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
class Lift(hardwareMap: HardwareMap, val arm: Arm? = null) : SubsystemBase() {
    val lift: DcMotorEx by lazy {
        hardwareMap["slides"] as DcMotorEx
    }

    val controller: PIDController = PIDController(p, i, d)
    val position: Int
        get() = lift.currentPosition
    var target: Double = 10.0

    val positionInches
        get() = position / ticksToInches
    val maxLiftPosition: Vec2d
        get() = if (arm == null) Vec2d(Double.POSITIVE_INFINITY, extensionLimit) else Vec2d(
            cos(arm.angle),
            sin(arm.angle)
        ) * (extensionLimit / cos(arm.angle))

    val maxLiftDistance: Double
        get() = if (arm == null) Double.POSITIVE_INFINITY else Vec2d(0, 0).distanceTo(maxLiftPosition)

    init {
        lift.direction = DcMotorSimple.Direction.FORWARD
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun periodic() {
        run()
    }

    fun run() {
        controller.setPID(p, i, d)
        val clampedTarget = if (!useExtensionLimit) target else min(maxLiftDistance, target)
        val pid = controller.calculate(position.toDouble(), clampedTarget.toDouble())
        val ff = if (arm == null) f else sin(arm.angle) * f

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
        var base = 10;

        @JvmField
        var pickup = 500;

        @JvmField
        var lowBasket = 1150;

        @JvmField
        var highBasket = 1150;

        @JvmField
        var ticksToInches = 1.0 // need to tune

        @JvmField
        var useExtensionLimit = false // enable after tuned

        @JvmField
        var extensionLimit = 40.0 // need to measure
    }
}