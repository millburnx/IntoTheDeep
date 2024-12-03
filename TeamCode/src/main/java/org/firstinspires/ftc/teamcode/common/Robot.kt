package org.firstinspires.ftc.teamcode.common

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class Robot(val opMode: OpMode) {
    val hardware: HardwareMap by lazy { opMode.hardwareMap }
    val drive by lazy { Drive(this) }
    val additionalSubsystems: List<Subsystem> = listOf()
    val subsystems: List<Subsystem> = listOf(
        drive, *additionalSubsystems.toTypedArray()
    )

    fun init() {
        subsystems.forEach {
            it.init() // triggers lazy loader
        }
    }
}