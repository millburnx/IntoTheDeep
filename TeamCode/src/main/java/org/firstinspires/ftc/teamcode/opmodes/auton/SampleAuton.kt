package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.millburnx.utils.TSV
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.drive.RelativeDrive
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDCommand
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.outtakeDropArmDelay
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.transferClawDelay
import java.io.File

@Autonomous(name = "Sample Auton", preselectTeleOp = "Basic Teleop")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
class SampleAuton : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    override fun initialize() {
        super.initialize()
        robot.drive.pose = Pose2d(startingX, startingY, startingHeading)
        val commands = mutableListOf<Command>()
        robot.outtake.arm.periodic()
        robot.outtake.claw.close()
        robot.outtake.claw.periodic()
        robot.intake.arm.periodic()
        robot.intake.diffy.periodic()
        robot.intake.claw.periodic()

        val grab = {
            SequentialCommandGroup(
                robot.intake.grab(),
                ParallelCommandGroup(
                    robot.intake.retract(),
                    robot.outtake.open(),
                    robot.outtake.base(),
                ),
                robot.outtake.close(),
                WaitCommand(transferClawDelay),
                robot.intake.open(),
            )
        }

        val up = {
            SequentialCommandGroup(
                InstantCommand({
                    robot.outtake.arm.state = OuttakeArmPosition.BASKET
                    robot.outtake.wrist.state = OuttakeWristPosition.BASKET
                }, robot.outtake),
                SlidesCommand(robot.outtake.slides, Slides.highBasket)
            )
        }

        val down = {
            SequentialCommandGroup(
                ParallelCommandGroup(
                    InstantCommand({
                        robot.outtake.arm.state = OuttakeArmPosition.BASE
                        robot.outtake.wrist.state = OuttakeWristPosition.BASE
                    }, robot.outtake.arm, robot.outtake.wrist),
                    WaitCommand(outtakeDropArmDelay),
                ),
                SlidesCommand(robot.outtake.slides, Slides.min)
            )
        }

        val drop = {
            SequentialCommandGroup(
                robot.outtake.open(),
                WaitCommand(250),
                ParallelCommandGroup(
                    ParallelCommandGroup(
                        InstantCommand({
                            robot.outtake.arm.state = OuttakeArmPosition.BASE
                            robot.outtake.wrist.state = OuttakeWristPosition.BASE
                        }, robot.outtake.arm, robot.outtake.wrist),
                        WaitCommand(outtakeDropArmDelay),
                    ),
                    PIDCommand(robot, Pose2d(basketX, basketY, -45.0))
                )
            )
        }

        commands.add(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    up(),
                    PIDCommand(robot, Pose2d(basketX, basketY, -45.0))
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(-0.3, 0.0, 0.0)).withTimeout(basketDuration),
                drop(),
                ParallelCommandGroup(
                    down(),
                    PIDCommand(robot, Pose2d(sample1X, sample1Y, 0.0), tolerance = Pose2d(1.0, 1.0, 7.5)),
                ),
                robot.intake.extend(),
                WaitCommand(2000),
                grab(),
                ParallelCommandGroup(
                    up(),
                    PIDCommand(robot, Pose2d(basketX, basketY, -45.0))
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(-0.3, 0.0, 0.0)).withTimeout(basketDuration),
                drop(),
                ParallelCommandGroup(
                    down(),
                    PIDCommand(robot, Pose2d(sample2X, sample2Y, 0.0), tolerance = Pose2d(1.0, 1.0, 7.5)),
                ),
                robot.intake.extend(),
                WaitCommand(2000),
                grab(),
                ParallelCommandGroup(
                    up(),
                    PIDCommand(robot, Pose2d(basketX, basketY, -45.0))
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(-0.3, 0.0, 0.0)).withTimeout(basketDuration),
                drop(),
                ParallelCommandGroup(
                    down(),
                    PIDCommand(robot, Pose2d(sample3X, sample3Y, sample3H), tolerance = Pose2d(1.0, 1.0, 7.5))
                ),
                robot.intake.extend(),
                InstantCommand({ robot.intake.diffy.roll = Diffy.roll45 }),
                WaitCommand(2000),
                grab(),
                ParallelCommandGroup(
                    up(),
                    PIDCommand(robot, Pose2d(basketX, basketY, -45.0))
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(-0.3, 0.0, 0.0)).withTimeout(basketDuration),
                drop(),
                down(),
            )
        )

        schedule(SequentialCommandGroup(*commands.toTypedArray()))
    }

    companion object {
        @JvmField
        var startingX = -60.0

        @JvmField
        var startingY = 17.0

        @JvmField
        var startingHeading = 0.0

        @JvmField
        var sample1X = -47.25

        @JvmField
        var sample1Y = 50.5

        @JvmField
        var sample2X = -47.25

        @JvmField
        var sample2Y = 60.5

        @JvmField
        var sample3X = -40.25

        @JvmField
        var sample3Y = 55.0

        @JvmField
        var sample3H = 45.0

        @JvmField
        var basketX = -49.0

        @JvmField
        var basketY = 50.0

        @JvmField
        var basketDuration: Long = 1000
    }

    fun loadPoints(file: String): List<Pose2d> {
        val csv = TSV.bufferedRead(File("${Environment.getExternalStorageDirectory().path}/Paths/$file.tsv"))
        val points: MutableList<Pose2d> = mutableListOf()
        for (item in csv) {
            points.add(Pose2d(item[0].toDouble(), item[1].toDouble(), item[2].toDouble()))
        }
        return points
    }

    fun pointToPath(points: List<Pose2d>): List<PIDCommand> {
        return points.map { PIDCommand(robot, it) }
    }

    override fun exec() {
    }
}