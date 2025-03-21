package org.firstinspires.ftc.teamcode.common

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Outtake
import org.firstinspires.ftc.teamcode.common.utils.DeltaTime
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.TelemetryManager

open class Robot(
    val opMode: OpMode,
) : SubsystemBase() {
    val telemetryManager by lazy { TelemetryManager(this) }
    val telemetry by lazy { telemetryManager.telemetry }

    var isRed: Boolean = false
    var doYellow: Boolean = false
    var useTeleopHold: Boolean = false

    val matchTimer by lazy { ElapsedTime() }

    val gp1: Gamepad by lazy { opMode.gamepad1 }
    val gp2: Gamepad by lazy { opMode.gamepad2 }

    val hardware: HardwareMap by lazy { opMode.hardwareMap }
    val hubs: MutableList<LynxModule> by lazy { hardware.getAll(LynxModule::class.java) }
    open val drive by lazy { Drive(this) }
    open val intake: Intake by lazy { Intake(this) }
    open val outtake: Outtake by lazy { Outtake(this) }
    open val additionalSubsystems: List<Subsystem> = listOf()
    open val subsystems: List<Subsystem> by lazy {
        listOf(
            drive,
            intake,
            outtake,
        ) + additionalSubsystems
    }
    val deltaTime = DeltaTime()

    open fun init() {
        hubs.forEach { it.bulkCachingMode = BulkCachingMode.MANUAL }
        subsystems.forEach {
            it.init() // triggers lazy loader
        }
    }

    val macros by lazy { Macros(this) }
}
