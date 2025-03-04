package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo.Direction
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.abs

class CachedServo(
    hwMap: HardwareMap,
    name: String,
    isAxon: Boolean = false,
    var cacheThreshold: Double = 0.0,
    isForward: Boolean = true,
) {
    private val servo = (hwMap[name] as ServoImplEx).apply { init(isForward, isAxon) }

    var isForward: Boolean = isForward
        set(value) {
            if (field == value) return
            field = value
            servo.direction = if (value) Direction.FORWARD else Direction.REVERSE
        }

    // this should essentially control the servo's position, the servo should not be modified in any other way
    var position: Double = 0.0
        set(value) {
            val clamped = value.clamp(0.0, 1.0)
            if (abs(clamped - field) < cacheThreshold) return
            servo.position = clamped
            field = clamped
        }
}

class CachedCRServo(
    hwMap: HardwareMap,
    name: String,
    val cacheThreshold: Double = 0.0,
    isForward: Boolean = true,
) {
    private val servo = (hwMap[name] as CRServoImplEx).apply { init(isForward) }

    var isForward: Boolean = isForward
        set(value) {
            if (field == value) return
            field = value
            servo.direction = if (value) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
        }

    // this should essentially control the motor's power, the motor should not be modified in any other way
    var power: Double = 0.0
        set(value) {
            val clamped = value.clamp(-1.0, 1.0)
            if (abs(clamped - field) < cacheThreshold) return
            servo.power = clamped
            field = clamped
        }
}
