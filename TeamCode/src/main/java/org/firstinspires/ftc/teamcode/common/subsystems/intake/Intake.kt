package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop.Companion.baseIntakeDuration
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop.Companion.intakePickupArmDelay
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop.Companion.intakePickupClawDelay

class Intake(
    val robot: Robot,
) : Subsystem() {
    val linkage: Linkage = Linkage(robot)
    val arm: IntakeArm = IntakeArm(robot)
    val diffy: Diffy = Diffy(robot)
    val claw: IntakeClaw = IntakeClaw(robot)
    val subsystems: List<Subsystem> = listOf(linkage, arm, diffy, claw)

    override fun init() {
        subsystems.forEach { it.init() }
    }

    fun extend() =
        ParallelCommandGroup(
            linkage.extend(),
            arm.extended(),
            diffy.hover(),
        )

    fun extendAsync() =
        ParallelCommandGroup(
            linkage.extendAsync(),
            arm.extended(),
            diffy.hover(),
        )

    fun baseExtend() =
        ParallelCommandGroup(
            WaitCommand(baseIntakeDuration),
            baseExtendAsync(),
        )

    fun baseExtendAsync() =
        ParallelCommandGroup(
            arm.extended(),
            diffy.hover(),
        )

    fun retract() =
        ParallelCommandGroup(
            linkage.retract(),
            arm.base(),
            diffy.transfer(),
        )

    fun retractAsync() =
        ParallelCommandGroup(
            linkage.retractAsync(),
            arm.base(),
            diffy.transfer(),
        )

    fun baseRetract() =
        ParallelCommandGroup(
            WaitCommand(baseIntakeDuration),
            baseRetractAsync(),
        )

    fun baseRetractAsync() =
        ParallelCommandGroup(
            arm.base(),
            diffy.transfer(),
        )

    fun grab() =
        SequentialCommandGroup(
            arm.floor(),
            diffy.pickup(),
            WaitCommand(intakePickupArmDelay),
            close(),
            WaitCommand(intakePickupClawDelay),
        )

    fun open() = InstantCommand(claw::open, claw)

    fun close() = InstantCommand(claw::close, claw)
}
