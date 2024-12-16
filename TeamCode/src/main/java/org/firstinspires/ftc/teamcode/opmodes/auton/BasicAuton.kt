package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDManager
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d

class AutonRobot(opMode: OpMode) : Robot(opMode) {
    val pidManager = PIDManager(this)
    override val additionalSubsystems = listOf(pidManager)
}

@Autonomous(name = "Basic Auton")
@Config
@SuppressWarnings("detekt:MagicNumber")
class BasicAuton : OpMode() {
    override val robot = AutonRobot(this)

    override fun initialize() {
        super.initialize()
        val commands = mutableListOf<Command>()
        commands.add(PIDCommand(robot, Pose2d(-36.0, 24.0, 90.0)))
        commands.add(
            SequentialCommandGroup(
                PIDCommand(robot, Pose2d(-48.0, 48.0, 135.0)),
                SlidesCommand(robot.outtake.slides, Slides.max)
            )
        )
        commands.add(WaitCommand(2000))
        commands.add(
            SequentialCommandGroup(
                PIDCommand(robot, Pose2d(-60.0, -48.0, 90.0)),
                SlidesCommand(robot.outtake.slides, Slides.min)
            )
        )
        schedule(*commands.toTypedArray())
    }

    override fun exec() {
        TODO("Not yet implemented")
    }
}