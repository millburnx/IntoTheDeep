package org.firstinspires.ftc.teamcode.common.utils

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.absoluteValue

class AxonCR(
    hardwareMap: HardwareMap,
    servoName: String,
    encoderName: String,
    val isForward: Boolean = true,
) : Subsystem() {
    val servo = (hardwareMap[servoName] as CRServoImplEx).apply { init(isForward) }
    val encoder = hardwareMap[encoderName] as AnalogInput

    private var lastRawPosition = 0.0
    private var rotations = 0.0
    val rawPosition
        get() = encoder.voltage / 3.3

    val position
        get() = rawPosition + rotations

    override fun periodic() {
        val deltaRawPosition = rawPosition - lastRawPosition
        if (deltaRawPosition.absoluteValue > .5) {
            if (deltaRawPosition < 0) {
                rotations += 1
            } else {
                rotations -= 1
            }
        }

        lastRawPosition = rawPosition
    }

    var power = 0.0
        set(value) {
            if (value == field) return
            field = value
            servo.power = value
        }
}
