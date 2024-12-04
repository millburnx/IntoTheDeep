package org.firstinspires.ftc.teamcode.common

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.common.utils.DeltaTime
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class Robot(val opMode: OpMode) : SubsystemBase() {
    val hardware: HardwareMap by lazy { opMode.hardwareMap }
    val hubs by lazy { hardware.getAll<LynxModule?>(LynxModule::class.java) }
    val drive by lazy { Drive(this) }
    val additionalSubsystems: List<Subsystem> = listOf()
    val subsystems: List<Subsystem> = listOf(
        drive, *additionalSubsystems.toTypedArray()
    )
    val deltaTime = DeltaTime()

    fun init() {
        hubs.forEach { it.bulkCachingMode = BulkCachingMode.AUTO }
        subsystems.forEach {
            it.init() // triggers lazy loader
        }
    }
}