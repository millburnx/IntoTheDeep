package org.firstinspires.ftc.teamcode.common.utils

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

class CachedMotor(
    val hwMap: HardwareMap,
    val name: String,
    var cacheThreshold: Double = 0.0,
    isBrake: Boolean = false,
    isForward: Boolean = true,
) {
    private val motor = (hwMap[name] as DcMotorEx).apply { init(isForward, isBrake) }

    var isForward: Boolean = isForward
        set(value) {
            if (field == value) return
            field = value
            motor.direction = if (value) Direction.FORWARD else Direction.REVERSE
        }

    var isBrake: Boolean = isBrake
        set(value) {
            if (field == value) return
            field = value
            motor.zeroPowerBehavior = if (value) ZeroPowerBehavior.BRAKE else ZeroPowerBehavior.FLOAT
        }

    // this should essentially control the motor's power, the motor should not be modified in any other way
    var power: Double = 0.0
        set(value) {
            if (abs(value - field) < cacheThreshold) return
            motor.power = value
            field = value
        }

    val position: Double
        get() = motor.currentPosition.toDouble()
}
