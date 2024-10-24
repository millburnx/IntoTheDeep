package org.firstinspires.ftc.teamcode.common.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

class Intake(hardwareMap: HardwareMap) : SubsystemBase() {
    val servo: CRServo by lazy {
        hardwareMap["Intake"] as CRServo
    }

    fun intake() {
        servo.power = 1.0
    }

    fun outtake() {
        servo.power = -1.0
    }

    fun stop() {
        servo.power = 0.0
    }
}