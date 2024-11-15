package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

@Config
class Intake(hardwareMap: HardwareMap) : SubsystemBase() {
    val servo: Servo by lazy {
        hardwareMap["Intake"] as Servo
    }
    var open: Boolean = true

    init {
//        (servo as ServoImplEx).pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)
    }

    fun fullOpen() {
        open = true
        servo.position = fullOpenPosition
    }

    fun open() {
        open = true
        servo.position = openPosition
    }

    fun close() {
        open = false
        servo.position = closedPosition
    }

    fun toggle() {
        open = !open
        if (open) open() else close()
    }

    companion object {
        @JvmField
        var fullOpenPosition: Double = 0.9

        @JvmField
        var openPosition: Double = 0.8

        @JvmField
        var closedPosition: Double = 0.6
    }
}