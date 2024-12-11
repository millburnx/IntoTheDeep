package org.firstinspires.ftc.teamcode.common

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Outtake
import org.firstinspires.ftc.teamcode.common.utils.DeltaTime
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

open class Robot(val opMode: OpMode) : SubsystemBase() {
    val telemetry = MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().telemetry)

    val hardware: HardwareMap by lazy { opMode.hardwareMap }
    val hubs by lazy { hardware.getAll<LynxModule?>(LynxModule::class.java) }
    val drive by lazy { Drive(this) }
    val intake: Intake by lazy { Intake(this) }
    val outtake: Outtake by lazy { Outtake(this) }
    val additionalSubsystems: List<Subsystem> = listOf()
    open val subsystems: List<Subsystem> = listOf(
        drive, intake, outtake, *additionalSubsystems.toTypedArray()
    )
    val deltaTime = DeltaTime()

    open fun init() {
        hubs.forEach { it.bulkCachingMode = BulkCachingMode.AUTO }
        subsystems.forEach {
            it.init() // triggers lazy loader
        }
    }
}