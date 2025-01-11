package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.drive.RelativeDrive
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDManager
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.specScoreDelay

class AutonRobot(opMode: OpMode) : Robot(opMode) {
    val pidManager = PIDManager(this)
    override val additionalSubsystems = listOf(pidManager)
}

@Autonomous(name = "Basic Auton")
@Config
@SuppressWarnings("detekt:MagicNumber")
class BasicAuton : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    override fun initialize() {
        super.initialize()
        robot.drive.pose = Pose2d(startingX, startingY, startingHeading)
        val commands = mutableListOf<Command>()
        robot.outtake.claw.close()
        robot.outtake.claw.periodic()
        robot.intake.arm.state = IntakeArmPosition.SPECIMEN
        robot.intake.arm.periodic()
        robot.intake.diffy.roll = Diffy.specimenRoll
        robot.intake.diffy.pitch = Diffy.specimenPitch
        robot.intake.diffy.periodic()
        commands.add(
            SequentialCommandGroup(
                PIDCommand(robot, Pose2d(-32.0, -2.0, 180.0)),
                ParallelDeadlineGroup(
                    WaitCommand(specScoreDelay),
                    InstantCommand({
                        robot.outtake.arm.state = OuttakeArmPosition.SPECIMEN
                        robot.outtake.wrist.state = OuttakeWristPosition.SPECIMEN
                    }, robot.outtake),
                    RelativeDrive(robot.drive, robot.pidManager, Pose2d(0.3, 0.0, 0.0)),
                ),
                SlidesCommand(robot.outtake.slides, Slides.highRung),
                robot.outtake.open(),
                ParallelCommandGroup(
                    robot.outtake.base()
                ),
                PIDCommand(robot, Pose2d(-32.0, -4.0, 180.0)),
                PIDCommand(robot, Pose2d(sweepX1, sweepY1, -45.0)),
                robot.intake.sweepExtend(),
                PIDCommand(robot, Pose2d(sweepX2, sweepY2, -120.0)),
                ParallelCommandGroup(
                    InstantCommand({ robot.intake.arm.state = IntakeArmPosition.SPECIMEN }),
                    WaitCommand(750)
                ),
                PIDCommand(robot, Pose2d(sweepX3, sweepY3, -45.0)),
                ParallelCommandGroup(
                    InstantCommand({ robot.intake.arm.state = IntakeArmPosition.SWEEP }),
                    WaitCommand(750)
                ),
                PIDCommand(robot, Pose2d(sweepX4, sweepY4, -120.0)),
                robot.intake.retract()
            )
        )
        schedule(SequentialCommandGroup(*commands.toTypedArray()))
    }

    override fun exec() {
    }

    companion object {
        @JvmField
        var sweepX1 = -33.0

        @JvmField
        var sweepY1 = -28.0

        // --

        @JvmField
        var sweepX2 = -40.0

        @JvmField
        var sweepY2 = sweepY1

        // --

        @JvmField
        var sweepX3 = sweepX1

        @JvmField
        var sweepY3 = -38.0

        // --

        @JvmField
        var sweepX4 = sweepX2

        @JvmField
        var sweepY4 = sweepY3

        @JvmField
        var startingX = -60.0

        @JvmField
        var startingY = -17.0

        @JvmField
        var startingHeading = 0.0
    }
}