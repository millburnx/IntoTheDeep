package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.millburnx.utils.TSV
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.drive.RelativeDrive
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDManager
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.teleop.ControlRewrite.Companion.intakeLoweringDuration
import org.firstinspires.ftc.teamcode.opmodes.teleop.ControlRewrite.Companion.outtakeFlipDelay
import org.firstinspires.ftc.teamcode.opmodes.teleop.ControlRewrite.Companion.specimenCloseDuration
import org.firstinspires.ftc.teamcode.opmodes.teleop.ControlRewrite.Companion.transferClawDelay
import java.io.File

class AutonRobot(
    opMode: OpMode,
) : Robot(opMode) {
    val pidManager = PIDManager(this)
    override val additionalSubsystems = listOf(pidManager)

    override fun init() {
        imu.resetYaw()
        super.init()
    }
}

@Autonomous(name = "Specimen Auton", preselectTeleOp = "Basic Teleop")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
class SpecimenAuton : OpMode() {
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

        fun extend() =
            SequentialCommandGroup(
                robot.intake.open(),
                robot.intake.extend(),
            )

        fun grab() =
            SequentialCommandGroup(
                robot.intake.grab(),
                robot.outtake.open(),
                robot.intake.retract(),
            )

        fun transfer() =
            SequentialCommandGroup(
                robot.outtake.close(),
                WaitCommand(transferClawDelay),
                robot.intake.open(),
                WaitCommand(outtakeFlipDelay),
            )

        fun drop() =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    robot.outtake.arm.human(),
                    robot.outtake.wrist.human(),
                    WaitCommand(outtakeArmDuration),
                ),
                robot.outtake.open(),
                ParallelCommandGroup(
                    robot.outtake.base(),
                    WaitCommand(outtakeArmDuration),
                ),
            )

        fun pickupSamples() =
            SequentialCommandGroup(
                PIDCommand(robot, Pose2d(Vec2d(sample1X, sample1Y), sample1H)),
                extend(),
                grab(),
                ParallelCommandGroup(
                    SequentialCommandGroup(
                        PIDCommand(robot, Pose2d(Vec2d(sample2X, sample2Y), sample2H)),
                    ),
                    SequentialCommandGroup(
                        transfer(),
                        ParallelCommandGroup(
                            extend(),
                            drop(),
                        ),
                    ),
                ),
                ParallelCommandGroup(
                    grab(),
                    ParallelCommandGroup(
                        robot.outtake.base(),
                        WaitCommand(outtakeArmDuration),
                    ),
                ),
                ParallelCommandGroup(
                    SequentialCommandGroup(
                        PIDCommand(robot, Pose2d(Vec2d(sample3X, sample3Y), sample3H)),
                    ),
                    SequentialCommandGroup(
                        transfer(),
                        ParallelCommandGroup(
                            extend(),
                            drop(),
                        ),
                    ),
                ),
                ParallelCommandGroup(
                    grab(),
                    ParallelCommandGroup(
                        robot.outtake.base(),
                        WaitCommand(outtakeArmDuration),
                    ),
                ),
                transfer(),
                drop(),
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

        fun specimenFlip() =
            SequentialCommandGroup(
                robot.outtake.close(),
                WaitCommand(specimenCloseDuration),
                SlidesCommand(robot.outtake.slides, Slides.wall),
                ParallelCommandGroup(
                    robot.outtake.arm.specimen(),
                    robot.outtake.wrist.specimen(),
                ),
            )

        fun scoreSpec() =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    PIDCommand(robot, Pose2d(Vec2d(pickupX, pickupY), -180.0)),
                    specimenPickup(),
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(pickupPower, 0.0, 0.0)).withTimeout(pickupDuration),
                ParallelCommandGroup(
                    specimenFlip(),
                    PIDCommand(robot, Pose2d(Vec2d(scoreX, scoreY), -180.0)),
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(scorePower, 0.0, 0.0)).withTimeout(scoreDuration),
                WaitCommand(humanDuration),
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
                pickupSamples(),
                scoreSpec(),
                scoreSpec(),
                scoreSpec(),
                scoreSpec(),
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

    override fun exec() {
    }

    companion object {
        @JvmField
        var startingX = -60.0

        @JvmField
        var startingY = -7.0

        @JvmField
        var startingHeading = 0.0

        @JvmField
        var scoreX = -36.0

        @JvmField
        var scoreY = 0.0

        @JvmField
        var scorePower = -0.5

        @JvmField
        var scoreDuration: Long = 500

        @JvmField
        var scoringDuration: Long = 750

        @JvmField
        var sample1X = -46.0

        @JvmField
        var sample1Y = -46.0

        @JvmField
        var sample1H = 0.0

        @JvmField
        var sample2X = -46.0

        @JvmField
        var sample2Y = -54.0

        @JvmField
        var sample2H = 0.0

        @JvmField
        var sample3X = -44.5

        @JvmField
        var sample3Y = -54.5

        @JvmField
        var sample3H = -30.0

        @JvmField
        var outtakeArmDuration = 1000L

        @JvmField
        var waitDuration = 750L

        @JvmField
        var pickupX = -60.0

        @JvmField
        var pickupY = -36.0

        @JvmField
        var pickupDuration = 500L

        @JvmField
        var pickupPower = 0.5

        @JvmField
        var humanDuration = 500L
    }
}
