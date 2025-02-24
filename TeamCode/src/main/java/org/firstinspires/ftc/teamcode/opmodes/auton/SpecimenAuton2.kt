package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.millburnx.utils.Path
import com.millburnx.utils.TSV
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.drive.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.commands.drive.RelativeDrive
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDCommand
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.teleop.ControlRewrite.Companion.intakeLoweringDuration
import org.firstinspires.ftc.teamcode.opmodes.teleop.ControlRewrite.Companion.specimenCloseDuration
import java.io.File

@Autonomous(name = "Specimen Auton 2", preselectTeleOp = "Basic Teleop")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
class SpecimenAuton2 : OpMode() {
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

        fun pickupSamples() =
            SequentialCommandGroup(
                robot.intake.open(),
                PIDCommand(robot, Pose2d(Vec2d(sample1GrabX, sample1GrabY), sample1GrabH)),
                WaitCommand(tempDelay),
                robot.intake.extend(),
                robot.intake.grab(),
                PIDCommand(robot, Pose2d(Vec2d(sample1DropX, sample1DropY), sample1DropH)),
                WaitCommand(tempDelay),
                robot.intake.open(),
                PIDCommand(robot, Pose2d(Vec2d(sample2GrabX, sample2GrabY), sample2GrabH)),
                WaitCommand(tempDelay),
                robot.intake.extend(),
                robot.intake.grab(),
                PIDCommand(robot, Pose2d(Vec2d(sample2DropX, sample2DropY), sample2DropH)),
                WaitCommand(tempDelay),
                robot.intake.open(),
                PIDCommand(robot, Pose2d(Vec2d(sample3GrabX, sample3GrabY), sample3GrabH)),
                WaitCommand(tempDelay),
                robot.intake.extend(),
                robot.intake.grab(),
                PIDCommand(robot, Pose2d(Vec2d(sample3DropX, sample3DropY), sample3DropH)),
                WaitCommand(tempDelay),
                robot.intake.open(),
                robot.intake.retract(),
            )

        fun specimenPickup() =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    SlidesCommand(robot.outtake.slides, Slides.min),
                    ConditionalCommand(
                        ParallelCommandGroup(
                            robot.intake.arm.specimen(),
                            robot.intake.diffy.specimen(),
                            WaitCommand(intakeLoweringDuration),
                        ),
                        InstantCommand({}),
                        { robot.intake.arm.state != IntakeArmPosition.SPECIMEN },
                    ),
                ),
                ParallelCommandGroup(
                    robot.outtake.arm.pickup(),
                    robot.outtake.wrist.pickup(),
                ),
                robot.outtake.open(),
            )

        fun specimenGrab() =
            SequentialCommandGroup(
                robot.outtake.close(),
                WaitCommand(specimenCloseDuration),
            )

        fun specimenFlip() =
            SequentialCommandGroup(
                SlidesCommand(robot.outtake.slides, Slides.wall),
                ParallelCommandGroup(
                    robot.outtake.arm.specimen(),
                    robot.outtake.wrist.specimen(),
                ),
            )

        fun scoreSpec(offset: Int) =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    PIDCommand(robot, Pose2d(Vec2d(pickupX, pickupY), -180.0)),
                    specimenPickup(),
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(pickupPower, 0.0, 0.0)).withTimeout(pickupDuration),
                WaitCommand(humanDuration),
                specimenGrab(),
                ParallelCommandGroup(
                    specimenFlip(),
                    PIDCommand(robot, Pose2d(Vec2d(scoreX, scoreY + offset * scoreOffset), -180.0)),
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(scorePower, 0.0, 0.0)).withTimeout(scoreDuration),
                ParallelCommandGroup(
                    robot.outtake.arm.specimenScoring(),
                    WaitCommand(scoringDuration),
                ),
                ParallelCommandGroup(
                    robot.outtake.open(),
                    robot.outtake.base(),
                ),
            )

        commands.add(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    PIDCommand(robot, Pose2d(Vec2d(scoreX, scoreY), -180.0)),
                    SlidesCommand(robot.outtake.slides, Slides.wall),
                    robot.outtake.arm.specimen(),
                    robot.outtake.wrist.specimen(),
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(scorePower, 0.0, 0.0)).withTimeout(scoreDuration),
                ParallelCommandGroup(
                    robot.outtake.arm.specimenScoring(),
                    WaitCommand(scoringDuration),
                ),
                ParallelCommandGroup(
                    robot.outtake.open(),
                    robot.outtake.base(),
                ),
                ParallelCommandGroup(
//                    specimenPickup(),
                    pickupSamples(),
                ),
                scoreSpec(1),
                scoreSpec(2),
                scoreSpec(3),
                scoreSpec(4),
            ),
        )

        schedule(SequentialCommandGroup(*commands.toTypedArray()))
    }

    fun loadPoints(file: String): List<Pose2d> {
        val csv = TSV.bufferedRead(File("${Environment.getExternalStorageDirectory().path}/Paths/$file.tsv"))
        val points: MutableList<Pose2d> = mutableListOf()
        for (item in csv) {
            points.add(Pose2d(item[0].toDouble(), item[1].toDouble(), item[2].toDouble()))
        }
        return points
    }

    fun pointToPath(points: List<Pose2d>): List<PIDCommand> = points.map { PIDCommand(robot, it) }

    fun loadPath(file: String): Path {
        val rootDir = Environment.getExternalStorageDirectory()
        val filePath = "$rootDir/Paths/$file.tsv"
        val path =
            try {
                val loaded = Vec2d.loadList(File(filePath))
                println(loaded)
                loaded
            } catch (e: Error) {
                e.printStackTrace()
                println("$file.tsv not found")
                Path(listOf())
            }
        return path
    }

    fun pp(
        path: Path,
        heading: Double,
    ) = PurePursuitCommand(robot, heading, path.points)

    override fun exec() {
    }

    companion object {
        @JvmField
        var startingX = -60.0

        @JvmField
        var startingY = -7.0

        @JvmField
        var startingHeading = 180.0

        @JvmField
        var scoreX = -36.0

        @JvmField
        var scoreY = 0.0

        @JvmField
        var scorePower = -1.0

        @JvmField
        var scoreDuration: Long = 250

        @JvmField
        var scoringDuration: Long = 500

        @JvmField
        var pickupX = -56.0

        @JvmField
        var pickupY = -36.0

        @JvmField
        var pickupDuration = 500L

        @JvmField
        var humanDuration = 375L

        @JvmField
        var pickupPower = .5

        @JvmField
        var scoreOffset = 1.0

        @JvmField
        var tempDelay = 1000L

        // sample1

        @JvmField
        var sample1GrabX: Double = -42.0

        @JvmField
        var sample1GrabY: Double = -36.0

        @JvmField
        var sample1GrabH: Double = -45.0

        @JvmField
        var sample1DropX: Double = -42.0

        @JvmField
        var sample1DropY: Double = -36.0

        @JvmField
        var sample1DropH: Double = -135.0

        // sample2

        @JvmField
        var sample2GrabX: Double = -42.0

        @JvmField
        var sample2GrabY: Double = -44.0

        @JvmField
        var sample2GrabH: Double = -45.0

        @JvmField
        var sample2DropX: Double = -42.0

        @JvmField
        var sample2DropY: Double = -44.0

        @JvmField
        var sample2DropH: Double = -135.0

        // sample3

        @JvmField
        var sample3GrabX: Double = -42.0

        @JvmField
        var sample3GrabY: Double = -52.0

        @JvmField
        var sample3GrabH: Double = -45.0

        @JvmField
        var sample3DropX: Double = -42.0

        @JvmField
        var sample3DropY: Double = -52.0

        @JvmField
        var sample3DropH: Double = -135.0
    }
}
