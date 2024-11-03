package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.common.commands.ArmCommand
import org.firstinspires.ftc.teamcode.common.commands.LiftCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift

fun SpecimenUp(arm: Arm, lift: Lift, intake: Intake) = SequentialCommandGroup(
    ArmCommand(arm, Specimen.Arm1).withTimeout(500),
    ArmCommand(arm, Specimen.Arm3).withTimeout(500),
    LiftCommand(lift, Specimen.Lift1).withTimeout(500),
    LiftCommand(lift, Specimen.Lift2).withTimeout(500),
)

fun SpecimenDown(arm: Arm, lift: Lift, intake: Intake) = SequentialCommandGroup(
    ArmCommand(arm, Specimen.Arm2).withTimeout(500),
    LiftCommand(lift, Specimen.Lift1).withTimeout(1000),
    InstantCommand({ intake.open() }),
    WaitCommand(200),
)

fun SpecimenDown2(arm: Arm, lift: Lift, intake: Intake) = SequentialCommandGroup(
    LiftCommand(lift, 30).withTimeout(500),
    WaitCommand(300),
    ArmCommand(arm, Specimen.Arm1).withTimeout(500),
    ArmCommand(arm, Specimen.Arm0).withTimeout(500),
)

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
        return false
    }
}

@Config
object Specimen {
    @JvmField
    var Arm0: Int = 0

    @JvmField
    var Arm1: Int = 90

    @JvmField
    var Arm2: Int = 150

    @JvmField
    var Arm3: Int = 160

    @JvmField
    var Lift1: Int = 350

    @JvmField
    var Lift2: Int = 745
}