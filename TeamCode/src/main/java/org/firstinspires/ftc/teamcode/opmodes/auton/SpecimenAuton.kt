package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop.Companion.intakeLoweringDuration
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop.Companion.specimenCloseDuration
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleCameraRobot

open class AutonRobot(
    opMode: OpMode,
) : SampleCameraRobot(opMode) {
    override fun init() {
        super.init()
        imu.resetYaw()
    }
}

@Autonomous(name = "Specimen Auton", preselectTeleOp = "Main Teleop")
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

        fun pickupSamples() =
            SequentialCommandGroup(
                robot.drive.purePursuit("pre1", -180.0),
                robot.drive.pid(Pose2d(pushX, -40.0, -180.0)),
                robot.drive.purePursuit("pre2", -180.0),
                robot.drive.pid(Pose2d(pushX, -52.0, -180.0)),
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
                    robot.drive.pid(Pose2d(pickupX, pickupY, -180.0)),
                    specimenPickup(),
                ),
                robot.drive.relativeDrive(Pose2d(pickupPower, 0.0, 0.0)).withTimeout(
                    pickupDuration,
                ),
                WaitCommand(humanDuration),
                specimenGrab(),
                ParallelCommandGroup(
                    specimenFlip(),
                    robot.drive.pid(Pose2d(scoreX, scoreY + offset * scoreOffset, -180.0)),
                ),
                robot.drive.relativeDrive(Pose2d(scorePower, 0.0, 0.0)).withTimeout(
                    scoreDuration,
                ),
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
                    robot.drive.pid(Pose2d(scoreX, scoreY, -180.0)),
                    SlidesCommand(robot.outtake.slides, Slides.wall),
                    robot.outtake.arm.specimen(),
                    robot.outtake.wrist.specimen(),
                ),
                robot.drive.relativeDrive(Pose2d(scorePower, 0.0, 0.0)).withTimeout(
                    scoreDuration,
                ),
                ParallelCommandGroup(
                    robot.outtake.arm.specimenScoring(),
                    WaitCommand(scoringDuration),
                ),
                ParallelCommandGroup(
                    robot.outtake.open(),
                    robot.outtake.base(),
                ),
                ParallelCommandGroup(
                    specimenPickup(),
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
        var pushX = -52.0

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
    }
}
