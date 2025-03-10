package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleCameraRobot

open class AutonRobot(
    opMode: OpMode,
) : SampleCameraRobot(opMode) {
    override fun init() {
        super.init()
        drive.pinPoint.reset()
    }
}

@Autonomous(name = "Specimen Auton", preselectTeleOp = "Main Teleop Red")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
class SpecimenAuton : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    val currentCommands = mutableListOf<String>()

    override fun initialize() {
        super.initialize()

        Thread.sleep(200)

        robot.apply {
            drive.pinPoint.pinPoint.setPosition(
                Pose2D(
                    DistanceUnit.INCH,
                    -24.0,
                    -36.0,
                    AngleUnit.DEGREES,
                    -12.0,
                ),
            )
            Thread.sleep(200)
            telemetry.addData("starting pose", Pose2d(startingPose))
            telemetry.addData("Pose", drive.pose)
            telemetry.update()
            outtake.arm.periodic()
            outtake.claw.close()
            outtake.claw.periodic()
            intake.arm.periodic()
//            intake.diffy.periodic()
            intake.claw.periodic()

            fun readySpecimen() =
                namedCommand(
                    "readySpecimen",
                    SequentialCommandGroup(
                        outtake.slides.goTo(Slides.autonHighRung),
                        outtake.autonSpecimenPartial(),
                    ),
                )

            fun clipSpecimen() =
                namedCommand(
                    "clipSpecimen",
                    SequentialCommandGroup(
                        outtake.slides.goTo(Slides.autonHighRungScore),
                        outtake.autonSpecimenPartial(),
                    ),
                )

            fun releaseSpecimen() = outtake.open()

            fun sweepSample(
                startingPose: Array<Double>,
                endingPose: Array<Double>,
                sampleCount: Int,
            ) = namedCommand(
                "sweepSample$sampleCount",
                SequentialCommandGroup(
                    drive.pid(Pose2d(startingPose)),
                    robot.intake.sweep(),
                    drive.pid(Pose2d(endingPose)),
                    robot.intake.retract(),
                ),
            )

            fun sweepSamples() =
                namedCommand(
                    "sweepSamples",
                    SequentialCommandGroup(
                        sweepSample(sweepFirstStartingPose, sweepFirstEndingPose, 0),
                        sweepSample(sweepSecondStartingPose, sweepSecondEndingPose, 1),
                        sweepSample(sweepThirdStartingPose, sweepThirdEndingPose, 2),
                    ),
                )

            fun pickupSpecimen(specimen: Int) =
                namedCommand(
                    "pickupSpecimen$specimen",
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            outtake.open(),
                            outtake.slides.goTo(Slides.min),
                            outtake.specimenPickupPartial(),
                            drive.pid(Pose2d(pickupPose)),
                        ),
                        drive.relativeDrive(Pose2d(pickupPower, 0.0, 0.0), true).withTimeout(pickupDuration),
                        outtake.close(),
                        outtake.slides.goTo(Slides.wall),
                    ),
                )

            fun scoreSpecimen(specimen: Int) =
                namedCommand(
                    "scoreSpecimen$specimen",
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            readySpecimen(),
                            drive.pid(Pose2d(scoringPose)),
                        ),
                        drive.relativeDrive(Pose2d(scoringPower, 0.0, 0.0), true).withTimeout(scoringDuration),
                        clipSpecimen(),
                        releaseSpecimen(),
                    ),
                )

            fun scorePreloadSpecimen() = scoreSpecimen(0) // make its own pure pursuit path later

            fun pickupAndScoreSpecimen(specimen: Int) =
                SequentialCommandGroup(
                    pickupSpecimen(specimen),
                    scoreSpecimen(specimen),
                )

            fun park() = namedCommand("park", drive.pid(Pose2d(parkPose)))

            schedule(
                delayedSequential(
                    1000,
                    scorePreloadSpecimen(),
                    sweepSamples(),
                    pickupAndScoreSpecimen(1),
                    pickupAndScoreSpecimen(2),
                    pickupAndScoreSpecimen(3),
                    pickupAndScoreSpecimen(4),
                    park(),
                ),
            )
        }
    }

    fun namedCommand(
        name: String,
        command: Command,
    ) = ParallelDeadlineGroup(
        command,
        RunCommand({ currentCommands.add(name) }),
    )

    fun delayedSequential(
        delay: Long,
        vararg commands: Command,
    ) = SequentialCommandGroup(
        *commands
            .map {
                listOf(
                    WaitCommand(delay),
                    it,
                )
            }.flatten()
            .toTypedArray(),
    )

    override fun run() {
        currentCommands.clear()
        super.run()
    }

    override fun exec() {
        telemetry.addData("pid target", robot.drive.pidManager.target)
        telemetry.addData("pid at target", robot.drive.pidManager.atTarget())
        telemetry.addData("current commands", currentCommands.joinToString(", "))
        println(currentCommands.joinToString(", "))
    }

    companion object {
        @JvmField
        var startingPose = arrayOf(-62.0, 7.0, 0.0)

        @JvmField
        var parkPose = arrayOf(-62.0, 7.0, 0.0)

        // make this pickup and scoring into a pure pursuit path

        // <editor-fold desc="Pickup Config">
        @JvmField
        var pickupPose = arrayOf(-60.0, 7.0, 0.0)

        @JvmField
        var pickupPower = -.5

        @JvmField
        var pickupDuration = 500L
        // </editor-fold>

        // <editor-fold desc="Scoring Config">
        @JvmField
        var scoringPose = arrayOf(-36.0, 0.0, 0.0)

        @JvmField
        var scoringPower = .5

        @JvmField
        var scoringDuration = 500L
        // </editor-fold>

        // <editor-fold desc="Sweep Poses">
        @JvmField
        var sweepFirstStartingPose = arrayOf(-36.0, -36.0, -45.0)

        @JvmField
        var sweepFirstEndingPose = arrayOf(-48.0, -36.0, 45.0)

        @JvmField
        var sweepSecondStartingPose = arrayOf(-36.0, -48.0, -45.0)

        @JvmField
        var sweepSecondEndingPose = arrayOf(-48.0, -48.0, 45.0)

        @JvmField
        var sweepThirdStartingPose = arrayOf(-36.0, -60.0, -45.0)

        @JvmField
        var sweepThirdEndingPose = arrayOf(-38.0, -60.0, 45.0)
        // </editor-fold>
    }
}
