package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.intakeDuration
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.intakePickupArmDelay
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.intakePickupClawDelay
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.sweepDuration

class Intake(
    val robot: Robot,
) : Subsystem() {
    inner class JSONSubsystem : com.millburnx.jsoncommands.Subsystem {
        override val type = "Subsystem/Intake"

        override fun generate(): SubsystemBase = this@Intake
    }

    val linkage: Linkage = Linkage(robot)
    val arm: IntakeArm = IntakeArm(robot)
    val diffy: Diffy = Diffy(robot)
    val claw: IntakeClaw = IntakeClaw(robot)
    override val subsystems: List<Subsystem> = listOf(linkage, arm, diffy, claw)

    fun extend() = ExtendCommand(this)

    fun sweepExtend() = SweepExtendCommand(this)

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
            InstantCommand({
                intake.linkage.target = 1.0
                intake.arm.state = IntakeArmPosition.EXTENDED
                intake.diffy.pitch = Diffy.hoverPitch
                intake.diffy.roll = Diffy.hoverRoll
            }, intake.linkage, intake.arm, intake.diffy),
        )
        addRequirements(intake.linkage, intake.arm, intake.diffy)
    }
}

class SweepExtendCommand(
    intake: Intake,
) : SequentialCommandGroup() {
    init {
        addCommands(
            InstantCommand({
                intake.linkage.target = 1.0
                intake.arm.state = IntakeArmPosition.SWEEP
                intake.diffy.pitch = Diffy.sweepPitch
                intake.diffy.roll = Diffy.sweepRoll
            }, intake.linkage, intake.arm, intake.diffy),
            WaitCommand(sweepDuration),
        )
        addRequirements(intake.linkage, intake.arm, intake.diffy)
    }
}

class RetractCommand(
    intake: Intake,
) : SequentialCommandGroup() {
    init {
        addCommands(
            InstantCommand({
                intake.linkage.target = 0.0
                intake.arm.state = IntakeArmPosition.BASE
                intake.diffy.pitch = Diffy.transferPitch
                intake.diffy.roll = Diffy.transferRoll
            }, intake.linkage, intake.arm, intake.diffy),
            WaitCommand(intakeDuration),
        )
        addRequirements(intake.linkage, intake.arm, intake.diffy)
    }
}

class BarSideRetractCommand(
    intake: Intake,
) : SequentialCommandGroup() {
    init {
        addCommands(
            InstantCommand({
                intake.linkage.target = 0.0
                intake.arm.state = IntakeArmPosition.EXTENDED
                intake.diffy.pitch = Diffy.transferPitch
                intake.diffy.roll = Diffy.transferRoll
            }, intake.linkage, intake.arm, intake.diffy),
            WaitCommand(intakeDuration),
            InstantCommand({
                intake.arm.state = IntakeArmPosition.BASE
            }, intake.arm),
        )
        addRequirements(intake.linkage, intake.arm, intake.diffy)
    }
}

class GrabCommand(
    intake: Intake,
) : SequentialCommandGroup() {
    init {
        addCommands(
            InstantCommand({
                intake.arm.state = IntakeArmPosition.FLOOR
                intake.diffy.pitch = Diffy.pickupPitch
            }, intake),
            WaitCommand(intakePickupArmDelay),
            intake.close(),
            WaitCommand(intakePickupClawDelay),
        )
        addRequirements(intake.arm, intake.diffy, intake.claw)
    }
}
