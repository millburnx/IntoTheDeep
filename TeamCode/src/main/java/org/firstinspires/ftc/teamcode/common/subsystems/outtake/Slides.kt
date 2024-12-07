package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init
import kotlin.math.abs

class Slides(val robot: Robot) : Subsystem() {
    val leftLift: DcMotorEx = (robot.hardware["leftLift"] as DcMotorEx).apply { init() }
    val rightLift: DcMotorEx = (robot.hardware["rightLift"] as DcMotorEx).apply { init() }
    val pid = PIDController(kP, kI, kD)
    var target: Double = 0.0
        set(value) {
            field = value.coerceIn(min, max)
        }

    override fun periodic() {
        pid.setPID(kP, kI, kD)
        val power = pid.calculate(leftLift.currentPosition.toDouble(), target) + kF
        val error = target - leftLift.currentPosition

        // disable motors if lift is at base and target is also base
        // I think to ignore this for resetting lift, you can set the target to below base (good anyway in case of drifting upwards)
        // once you reset the encoder just set the target to base, and it should be fine
        // alternatively just use a limit switch to reset the encoder (!)
        if (abs(0 - target) < 5.0 && abs(error) < 10.0) {
            leftLift.power = 0.0
            rightLift.power = 0.0
        }

        leftLift.power = power
        rightLift.power = power

    }

    companion object {
        @JvmField
        var kP: Double = 0.03

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kF: Double = 0.01

        @JvmField
        var min: Double = 0.0

        @JvmField
        var max: Double = 2500.0
    }
}