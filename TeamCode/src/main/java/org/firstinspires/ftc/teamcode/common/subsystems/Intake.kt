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

    fun open() {
        servo.position = openPosition
    }

    fun close() {
        servo.position = closedPosition
    }

    companion object {
        @JvmField
        var openPosition: Double = 0.25

        @JvmField
        var closedPosition: Double = 0.0325
    }
}