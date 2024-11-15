package org.firstinspires.ftc.teamcode.common.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Park(hardwareMap: HardwareMap) : SubsystemBase() {
    val servo: Servo by lazy {
        hardwareMap["parkServo"] as Servo
    }

    var isParked: Boolean = false

    fun park() {
        isParked = true
    }

    fun idle() {
        isParked = false
    }

    override fun periodic() {
        if (isParked) {
            servo.position = 0.6
        } else {
            servo.position = 1.0
        }
    }
}