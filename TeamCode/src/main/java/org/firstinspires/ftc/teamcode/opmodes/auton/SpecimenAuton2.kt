package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.drive.RelativeDrive
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop.Companion.intakeLoweringDuration
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop.Companion.specimenCloseDuration

@Autonomous(name = "Specimen Auton 2", preselectTeleOp = "Main Teleop")
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
                robot.drive.pid(Pose2d(sample1GrabX, sample1GrabY, sample1GrabH)),
                WaitCommand(tempDelay),
                robot.intake.extend(),
                robot.intake.grab(),
                robot.drive.pid(Pose2d(sample1DropX, sample1DropY, sample1DropH)),
                WaitCommand(tempDelay),
                robot.intake.open(),
                robot.drive.pid(Pose2d(sample2GrabX, sample2GrabY, sample2GrabH)),
                WaitCommand(tempDelay),
                robot.intake.extend(),
                robot.intake.grab(),
                robot.drive.pid(Pose2d(sample2DropX, sample2DropY, sample2DropH)),
                WaitCommand(tempDelay),
                robot.intake.open(),
                robot.drive.pid(Pose2d(sample3GrabX, sample3GrabY, sample3GrabH)),
                WaitCommand(tempDelay),
                robot.intake.extend(),
                robot.intake.grab(),
                robot.drive.pid(Pose2d(sample3DropX, sample3DropY, sample3DropH)),
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
                ParallelCommandGroup(
                    robot.outtake.arm.specimen(),
                    robot.outtake.wrist.specimen(),
                    SlidesCommand(robot.outtake.slides, Slides.highRung),
                ),
            )

        fun scoreSpec(offset: Int) =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    robot.drive.pid(Pose2d(pickupX, pickupY, -180.0)),
                    specimenPickup(),
                ),
                RelativeDrive(robot.drive, robot.drive.pidManager, Pose2d(pickupPower, 0.0, 0.0)).withTimeout(
                    pickupDuration,
                ),
                WaitCommand(humanDuration),
                specimenGrab(),
                ParallelCommandGroup(
                    specimenFlip(),
                    robot.drive.pid(Pose2d(scoreX, scoreY + offset * scoreOffset, -180.0)),
                ),
                SlidesCommand(robot.outtake.slides, Slides.highRungScore),
                ParallelCommandGroup(
                    robot.outtake.open(),
                    robot.outtake.base(),
                ),
            )

        commands.add(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    robot.drive.pid(Pose2d(scoreX, scoreY, -180.0)),
                    specimenFlip(),
                ),
                SlidesCommand(robot.outtake.slides, Slides.highRungScore),
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
