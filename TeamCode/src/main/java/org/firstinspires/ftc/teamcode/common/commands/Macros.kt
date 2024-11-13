package org.firstinspires.ftc.teamcode.common.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandGroupBase
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift

fun SpecimenScore1(arm: Arm, lift: Lift, intake: Intake) = SequentialCommandGroup(
    ArmCommand(arm, Specimen.arm).withTimeout(1000),
)

fun SpecimenScore2(arm: Arm, lift: Lift, intake: Intake) = SequentialCommandGroup(
    LiftCommand(lift, Specimen.lift).withTimeout(1000),
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
object Specimen {
    @JvmField
    var arm: Int = 160

    @JvmField
    var lift: Int = 1150
}