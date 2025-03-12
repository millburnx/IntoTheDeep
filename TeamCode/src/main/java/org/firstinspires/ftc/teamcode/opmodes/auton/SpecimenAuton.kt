package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleCameraRobot

open class AutonRobot(
    opMode: OpMode,
) : SampleCameraRobot(opMode) {
    override fun init() {
        super.init()
    }
}

// @Autonomous(name = "Specimen Auton", preselectTeleOp = "Main Teleop Red")
@TeleOp(name = "Specimen Auton", group = "Auton")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
class SpecimenAuton : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    val currentCommands = mutableListOf<String>()

    override fun initialize() {
        super.initialize()

        robot.apply {
            drive.pinPoint.pinPoint.resetPosAndIMU()
            sleep(500)
            drive.pinPoint.update()
            drive.pinPoint.pose = Pose2d(startingPose)
            drive.pinPoint.update()
            telemetry.addData("starting pose", Pose2d(startingPose))
            telemetry.addData("Pose", drive.pose)
            telemetry.update()
            outtake.arm.state = OuttakeArmPosition.AUTON_SPECIMEN
            outtake.arm.periodic()
            outtake.wrist.state = OuttakeWristPosition.AUTON_SPECIMEN
            outtake.wrist.periodic()
            outtake.claw.close()
            outtake.claw.periodic()
            intake.arm.periodic()
            intake.claw.periodic()

            val tolerance =
                Pose2d(
                    PIDSettings.tolerance,
                    PIDSettings.headingTolerance,
                )

            fun readySpecimen() =
                namedCommand(
                    "readySpecimen",
                    ParallelCommandGroup(
                        outtake.slides.goTo(Slides.State.AUTON_HIGH_RUNG),
                        outtake.autonSpecimenPartial(),
                    ),
                )

            fun clipSpecimen() =
                namedCommand(
                    "clipSpecimen",
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            outtake.slides.goTo(Slides.State.AUTON_HIGH_RUNG_SCORE),
                            outtake.autonSpecimenPartial(),
                        ),
                        WaitCommand(clipSpecimenDuration),
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
                    drive.pid(Pose2d(startingPose), tolerance = tolerance * 2.0),
                    ConditionalCommand(
                        intake.sweepAsync(),
                        intake.sweep(),
                    ) { intake.linkage.target == 1.0 },
                    drive.pid(Pose2d(endingPose), tolerance = tolerance * 2.0),
                    intake.arm.extended(),
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
                            outtake.slides.goTo(Slides.State.BASE),
                            outtake.specimenPickupPartial(),
                            drive.pid(Pose2d(pickupPose), tolerance = tolerance * 2.0),
                        ),
                        drive.pid(Pose2d(pickupPoseDeep), useStuckDectector = true),
                        outtake.close(),
                        WaitCommand(pickupDuration),
                        outtake.slides.goTo(Slides.State.WALL),
                    ),
                )

            fun scoreSpecimen(specimen: Int) =
                namedCommand(
                    "scoreSpecimen$specimen",
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            readySpecimen(),
                            SequentialCommandGroup(
                                WaitUntilCommand {
                                    outtake.slides.position > Slides.autonHighRung / 2.0
                                },
                                drive.pid(Pose2d(scoringPose), tolerance = tolerance * 2.0),
                            ),
                        ),
                        drive.pid(Pose2d(scoringPoseDeep), useStuckDectector = true),
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
                    0,
                    scorePreloadSpecimen(),
                    InstantCommand({ println("Starting sample sweep @ ${matchTimer.seconds()}") }),
                    ParallelCommandGroup(
                        outtake.basePartial(),
                        outtake.slides.goTo(Slides.State.BASE),
                        sweepSamples(),
                    ),
                    InstantCommand({ println("Starting specimen scoring @ ${matchTimer.seconds()}") }),
                    ParallelCommandGroup(
                        intake.retract(),
                        SequentialCommandGroup(
                            pickupAndScoreSpecimen(1),
                            pickupAndScoreSpecimen(2),
                            pickupAndScoreSpecimen(3),
                            pickupAndScoreSpecimen(4),
                        ),
                    ),
                    ParallelCommandGroup(
                        outtake.base(),
                        park(),
                    ),
                ),
            )
        }
    }

    fun namedCommand(
        name: String,
        command: Command,
    ) = command.beforeStarting { currentCommands.add(name) }.whenFinished { currentCommands.remove(name) }
//    fun namedCommand(
//        name: String,
//        command: Command,
//    ) = command

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

    override fun exec() {
        robot.telemetry.addData("slide pos", robot.outtake.slides.position)
        robot.telemetry.addData("slide target", robot.outtake.slides.target)
        robot.telemetry.addData("slide state", robot.outtake.slides.state)
        robot.telemetry.addData("pid target", robot.drive.pidManager.target)
        robot.telemetry.addData("pid at target", robot.drive.pidManager.atTarget())
        robot.telemetry.addData("current commands", currentCommands.joinToString(", "))
        robot.telemetry.addData("time", robot.matchTimer.seconds())
        println(currentCommands.joinToString(", "))
    }

    companion object {
        @JvmField
        var startingPose = arrayOf(-62.0, -7.0, 0.0)

        @JvmField
        var parkPose = arrayOf(-74.0, -36.0, 0.0)

        // make this pickup and scoring into a pure pursuit path

        // <editor-fold desc="Pickup Config">
        @JvmField
        var pickupPose = arrayOf(-48.0, -36.0, 0.0)

        @JvmField
        var pickupPoseDeep = arrayOf(-64.0, -36.0, 0.0)

        @JvmField
        var pickupDuration: Long = 250
        // </editor-fold>

        // <editor-fold desc="Scoring Config">
        @JvmField
        var scoringPose = arrayOf(-40.0, 0.0, 0.0)

        @JvmField
        var scoringPoseDeep = arrayOf(-0.0, 0.0, 0.0)

        @JvmField
        var clipSpecimenDuration: Long = 500
        // </editor-fold>

        // <editor-fold desc="Sweep Poses">
        @JvmField
        var sweepFirstStartingPose = arrayOf(-38.0, -29.0, -45.0)

        @JvmField
        var sweepFirstEndingPose = arrayOf(-48.0, -29.0, -135.0)

        @JvmField
        var sweepSecondStartingPose = arrayOf(-39.0, -38.0, -45.0)

        @JvmField
        var sweepSecondEndingPose = arrayOf(-48.0, -38.0, -135.0)

        @JvmField
        var sweepThirdStartingPose = arrayOf(-38.0, -48.0, -45.0)

        @JvmField
        var sweepThirdEndingPose = arrayOf(-48.0, -44.0, -135.0)
        // </editor-fold>
    }
}
