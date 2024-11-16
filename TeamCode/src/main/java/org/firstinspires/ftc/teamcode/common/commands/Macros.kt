package org.firstinspires.ftc.teamcode.common.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandGroupBase
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift

fun SpecimenScore(arm: Arm, lift: Lift, intake: Intake) = SequentialCommandGroup(
    ArmCommand(arm, Specimen.arm).withTimeout(1000),
    WaitCommand(Specimen.delay),
    LiftCommand(lift, Specimen.lift1).withTimeout(1000),
)

fun SampleScore(arm: Arm, lift: Lift, intake: Intake) = SequentialCommandGroup(
    ArmCommand(arm, Sample.arm).withTimeout(1000),
    WaitCommand(Sample.delay),
    LiftCommand(lift, Sample.lift).withTimeout(1000),
)

fun ReturnToBase(arm: Arm, lift: Lift): CommandGroupBase {
    return SequentialCommandGroup(
        LiftCommand(lift, Lift.base),
        ArmCommand(arm, Arm.base),
    )
}

class RelativeDrive(val drive: Drive, val power: Double) : CommandBase() {
    init {
        addRequirements(drive)
    }

    override fun execute() {
        drive.robotCentric(power, 0.0, 0.0)
    }

    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        // we should add smt w/ odom to seem if we are moving at all
        // if we are fully still for a while then we're already hitting the wall
        return false
    }
}

@Config
object Sample {
    @JvmField
    var arm: Int = 115

    @JvmField
    var lift: Double = 1550.0

    @JvmField
    var delay: Long = 1000
}

@Config
object Specimen {
    @JvmField
    var arm: Int = 117

    @JvmField
    var lift1: Double = 1325.0

    @JvmField
    var delay: Long = 1000
}