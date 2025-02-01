package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class Outtake(
    val robot: Robot,
) : Subsystem() {
    inner class JSONSubsystem : com.millburnx.jsoncommands.Subsystem {
        override val type = "Subsystem/Outtake"

        override fun generate(): SubsystemBase = this@Outtake
    }

    val slides: Slides = Slides(robot)
    val arm: OuttakeArm = OuttakeArm(robot)
    val wrist: OuttakeWrist = OuttakeWrist(robot)
    val claw: OuttakeClaw = OuttakeClaw(robot)
    override val subsystems: List<Subsystem> = listOf(slides, arm, wrist, claw)

    override fun init() {
        subsystems.forEach { it.init() }
    }

    fun base() = BaseCommand(this)

    fun open() = InstantCommand(claw::open, claw)

    fun close() = InstantCommand(claw::close, claw)
}

class BaseCommand(
    outtake: Outtake,
) : SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                SlidesCommand(outtake.slides, Slides.min),
                ParallelCommandGroup(
                    InstantCommand({
                        outtake.arm.state = OuttakeArmPosition.BASE
                        outtake.wrist.state = OuttakeWristPosition.BASE
                    }, outtake.arm, outtake.wrist),
                ),
            ),
        )
        addRequirements(outtake.slides, outtake.arm, outtake.wrist)
    }
}
