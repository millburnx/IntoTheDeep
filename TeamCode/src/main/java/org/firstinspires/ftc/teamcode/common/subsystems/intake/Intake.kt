package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
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

    fun extend() = ExtendCommand(this)

    fun retract() = RetractCommand(this)

    fun barSideRetract() = BarSideRetractCommand(this)

    fun grab() = GrabCommand(this)

    fun open() = InstantCommand(claw::open, claw)

    fun close() = InstantCommand(claw::close, claw)
}

class ExtendCommand(
    intake: Intake,
) : SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                intake.linkage.extend(),
                intake.arm.extended(),
                intake.diffy.hover(),
            ),
        )
    }
}

class RetractCommand(
    intake: Intake,
) : SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                intake.linkage.retract(),
                intake.arm.base(),
                intake.diffy.transfer(),
            ),
        )
    }
}

class BarSideRetractCommand(
    intake: Intake,
) : SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                intake.linkage.retract(),
                intake.arm.extended(),
                intake.diffy.transfer(),
            ),
            intake.arm.base(),
        )
    }
}

class GrabCommand(
    intake: Intake,
) : SequentialCommandGroup() {
    init {
        addCommands(
            intake.arm.floor(),
            intake.diffy.pickup(),
            WaitCommand(intakePickupArmDelay),
            intake.close(),
            WaitCommand(intakePickupClawDelay),
        )
    }
}
