package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx

@Config
class Intake(hardwareMap: HardwareMap) : SubsystemBase() {
    val servo: Servo by lazy {
        hardwareMap["Intake"] as Servo
    }
    var open: Boolean = true

    init {
//        (servo.controller as ServoControllerEx).setServoPwmRange(500, 2500)
        (servo as ServoImplEx).pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)
    }

    fun open() {
        servo.position = openPosition
    }

    fun close() {
        servo.position = closedPosition
    }

    fun toggle() {
        open = !open
        servo.position = if (open) openPosition else closedPosition
    }

    companion object {
        @JvmField
        var openPosition: Double = 0.7

        @JvmField
        var closedPosition: Double = 0.4
    }
}