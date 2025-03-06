package org.firstinspires.ftc.teamcode.common.utils.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import kotlin.math.absoluteValue

class AxonCR(
    hardwareMap: HardwareMap,
    servoName: String,
    encoderName: String,
    val isForward: Boolean = true,
) : Subsystem() {
    val servo = CachedCRServo(hardwareMap, servoName, isForward = isForward)
    val encoder = hardwareMap[encoderName] as AnalogInput

    private var lastRawPosition = 0.0
    private var rotations = 0.0
    val rawPosition
        get() = encoder.voltage / 3.3 * if (isForward) 1.0 else -1.0

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
            servo.power = value
            field = value
        }
}
