package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.millburnx.utils.Path
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.conditionalCommand

@Autonomous(name = "Specimen Auton Push", preselectTeleOp = "Main Teleop Blue")
// @TeleOp(name = "Specimen Auton", group = "Auton")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
open class SpecimenAutonPush : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    val currentCommands = mutableListOf<String>()

    override fun initialize() {
        robot.apply {
            drive.pinPoint.pinPoint.resetPosAndIMU()
            sleep(500)
            drive.pinPoint.update()
            drive.pinPoint.pose = Pose2d(startingPose)
            drive.pinPoint.update()
            telemetry.addData("starting pose", Pose2d(startingPose))
            telemetry.addData("Pose", drive.pose)
            telemetry.update()
            outtake.arm.state = OuttakeArmPosition.BASE
            outtake.arm.periodic()
            outtake.wrist.state = OuttakeWristPosition.BASE
            outtake.wrist.periodic()
            outtake.claw.close()
            outtake.claw.periodic()
            intake.arm.state = IntakeArmPosition.BASE
            intake.diffy.state = Diffy.State.TRANSFER
            intake.arm.periodic()
            intake.claw.periodic()
            intake.diffy.resetDiffyRotations()

            super.initialize()

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

            fun releaseSpecimen() = outtake.open()

            fun pushSamples(): SequentialCommandGroup {
                val pre1 = robot.drive.purePursuit("pre1", -180.0, useStuckDectector = true)
                val pre2 = robot.drive.purePursuit("pre2", -180.0, useStuckDectector = true)
                val push1 = robot.drive.pid(Pose2d(pushX, -40.0, -180.0), useStuckDectector = true)
                val push2 = robot.drive.pid(Pose2d(pushX, -52.0, -180.0), useStuckDectector = true)
                return SequentialCommandGroup(
                    ParallelCommandGroup(
                        pre1,
                        WaitUntilCommand {
                            val pp = pre1.purePursuit
                            val pastSegment = pp.beziers.indexOf(pp.lastIntersection.line) >= 1
                            val pastT = pp.lastIntersection.t > 0.0
                            telemetry.addData("past segment", pastSegment)
                            telemetry.addData("past t", pastT)
                            return@WaitUntilCommand pastSegment && pastT
                        }.andThen(
                            InstantCommand({
                                pre1.speed = pushSlowSpeed
                            }),
                        ),
                    ),
                    InstantCommand({ println("Behind Sample 1 @ ${matchTimer.seconds()}") }),
                    ParallelCommandGroup(
                        push1,
                        WaitUntilCommand({ drive.pose.x < slowThreshold }).andThen(
                            InstantCommand({ push1.speed = slowSpeed }),
                        ),
                    ),
                    InstantCommand({ println("Pushed Sample 1 @ ${matchTimer.seconds()}") }),
                    ParallelCommandGroup(
                        pre2,
                        WaitUntilCommand {
                            val pp = pre2.purePursuit
                            val pastSegment = pp.beziers.indexOf(pp.lastIntersection.line) >= 1
                            val pastT = pp.lastIntersection.t > 0.0
                            telemetry.addData("past segment", pastSegment)
                            telemetry.addData("past t", pastT)
                            return@WaitUntilCommand pastSegment && pastT
                        }.andThen(
                            InstantCommand({
                                pre2.speed = pushSlowSpeed
                            }),
                        ),
                    ),
                    InstantCommand({ println("Behind Sample 2 @ ${matchTimer.seconds()}") }),
                    ParallelCommandGroup(
                        push2,
                        WaitUntilCommand({ drive.pose.x < slowThreshold }).andThen(
                            InstantCommand({ push2.speed = slowSpeed }),
                        ),
                    ),
                    InstantCommand({ println("Pushed Sample 2 @ ${matchTimer.seconds()}") }),
                )
            }

            fun pickupSpecimen(specimen: Int) =
                namedCommand(
                    "pickupSpecimen$specimen",
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            drive.pid(Pose2d(pickupPose), tolerance = tolerance * 2.0),
                            SequentialCommandGroup(
                                outtake.open(),
                                WaitCommand(500),
                                outtake.tightOpen(), // slightly faster cuz less servo movement time 💀
                            ),
                            SequentialCommandGroup(
                                ParallelCommandGroup(
                                    conditionalCommand(
                                        intake.linkage.retract(),
                                    ) { intake.linkage.target == 1.0 },
                                    ConditionalCommand(
                                        intake.arm.safe(),
                                        intake.arm.safe2(),
                                    ) { outtake.slides.position > Slides.highRung / 2 },
                                    intake.diffy.autonSpecimen(),
                                ),
                                ParallelCommandGroup(
                                    WaitCommand(armWait).andThen(
                                        outtake.autonSpecimenPickupPartial(),
                                        WaitCommand(625),
                                    ),
                                    WaitCommand(liftWait).andThen(
                                        ParallelCommandGroup(
                                            outtake.slides.goTo(Slides.State.BASE),
                                            WaitUntilCommand {
                                                outtake.slides.position < Slides.highRung
                                            }.andThen(
                                                intake.arm.safe2(),
                                            ),
                                        ),
                                    ),
                                ),
                            ),
                        ),
                        drive.pid(Pose2d(pickupPoseDeep), useStuckDectector = true, speed = grabSlowSpeed),
                        outtake.close(),
                        WaitCommand(pickupDuration),
                    ),
                )

            fun getScoreSpecimenPath(specimen: Int): Path {
                val pickupPos = Pose2d(pickupPose).position

                val scoringPos = Pose2d(scoringPose).position
                val scoringPosDeep = Pose2d(scoringPoseDeep).position

                val offset = Pose2d(scoringOffset).position * specimen

                return Path(
                    listOf(
                        pickupPos,
                        Vec2d(-50.0, pickupPos.y),
                    ) +
                        listOf(
                            Vec2d(-54, scoringPos.y) + offset,
                            scoringPos + offset,
                            Vec2d(-36, scoringPos.y) + offset,
                        ) +
                        listOf(
                            Vec2d(-36.0, 0.0) + offset,
                            scoringPosDeep + offset,
                        ),
                )
            }

            fun scoreSpecimen(specimen: Int): Command {
                val purePursuitPath =
                    drive.purePursuit(
                        getScoreSpecimenPath(specimen),
                        Pose2d(scoringPose).heading,
                        useStuckDectector = true,
                    )
                return namedCommand(
                    "scoreSpecimen$specimen",
                    SequentialCommandGroup(
                        intake.arm.safe(),
                        readySpecimen(),
                        ParallelCommandGroup(
                            namedCommand(
                                "pathing",
                                purePursuitPath,
                            ),
                            WaitUntilCommand {
                                val pp = purePursuitPath.purePursuit
                                val pastSegment = pp.beziers.indexOf(pp.lastIntersection.line) >= scoringSlowSegment
                                val pastT = pp.lastIntersection.t > scoringSlowT
                                telemetry.addData("past segment", pastSegment)
                                telemetry.addData("past t", pastT)
                                return@WaitUntilCommand pastSegment && pastT
                            }.andThen(
                                InstantCommand({
                                    purePursuitPath.speed = slowSpeed
                                }),
                            ),
                        ),
                        WaitCommand(clipSpecimenDuration),
                        releaseSpecimen(),
                    ),
                )
            }

            fun scorePreloadSpecimen() =
                namedCommand(
                    "scoreSpecimenPreload",
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            readySpecimen(),
                            WaitUntilCommand {
                                outtake.slides.position > Slides.autonHighRung * 1 / 4
                            }.andThen(
                                drive.pid(
                                    Pose2d(scoringPose),
                                    tolerance = tolerance * 2.0,
                                    speed = pushSlowSpeed,
                                ),
                            ),
                        ),
                        drive.pid(
                            Pose2d(scoringPoseDeep),
                            useStuckDectector = true,
                            speed = slowSpeed,
                        ),
                        WaitCommand(clipSpecimenDuration).andThen(
                            releaseSpecimen(),
                        ),
                    ),
                ) // make its own pure pursuit path later

            fun pickupAndScoreSpecimen(specimen: Int) =
                SequentialCommandGroup(
                    pickupSpecimen(specimen),
                    InstantCommand({ println("Picked up specimen $specimen @ ${matchTimer.seconds()}") }),
                    scoreSpecimen(specimen),
                    InstantCommand({ println("Scored specimen $specimen @ ${matchTimer.seconds()}") }),
                )

            fun park() = namedCommand("park", drive.pid(Pose2d(parkPose)))

            schedule(
                delayedSequential(
                    0,
                    scorePreloadSpecimen(),
                    InstantCommand({ println("Starting sample sweep @ ${matchTimer.seconds()}") }),
                    ParallelCommandGroup(
                        WaitCommand(500).andThen(
                            outtake.base(),
                        ),
                        pushSamples(),
                    ),
                    InstantCommand({ println("Starting specimen scoring @ ${matchTimer.seconds()}") }),
                    ParallelCommandGroup(
                        SequentialCommandGroup(
                            pickupAndScoreSpecimen(1),
                            pickupAndScoreSpecimen(2),
                            pickupAndScoreSpecimen(3),
//                            pickupAndScoreSpecimen(4),
                        ),
                    ),
                    ParallelCommandGroup(
                        park(),
                        WaitCommand(1000).andThen(
                            ParallelCommandGroup(
                                intake.arm.base(),
                                intake.diffy.transfer(),
                                outtake.slides.goTo(Slides.State.BASE),
                            ),
                        ),
                    ),
                ),
            )
        }
    }

    fun namedCommand(
        name: String,
        command: Command,
    ) = command.beforeStarting { currentCommands.add(name) }.whenFinished { currentCommands.remove(name) }

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
        println(currentCommands.joinToString(" | "))
    }

    companion object {
        @JvmField
        var startingPose = arrayOf(-64.0, -7.0, 180.0)

        @JvmField
        var parkPose = arrayOf(-54.0, -36.0, 180.0)

        @JvmField
        var pushSlowSpeed = .75

        @JvmField
        var grabSlowSpeed = .45

        @JvmField
        var slowSpeed = .5

        // make this pickup and scoring into a pure pursuit path

        // <editor-fold desc="Pickup Config">
        @JvmField
        var pickupPose = arrayOf(-50.0, -36.0, 180.0)

        @JvmField
        var pickupPoseDeep = arrayOf(-72.0, -36.0, 180.0)

        @JvmField
        var pickupDuration: Long = 125
        // </editor-fold>

        // <editor-fold desc="Scoring Config">
        @JvmField
        var scoringPose = arrayOf(-38.0, -2.0, 180.0)

        @JvmField
        var scoringSlowSegment = 1

        @JvmField
        var scoringSlowT = 0.5

        @JvmField
        var scoringPoseDeep = arrayOf(-20.0, -2.0, 180.0)

        @JvmField
        var scoringOffset = arrayOf(0.0, 2.0, 0.0)

        @JvmField
        var exitingScoring = arrayOf(-40.0, -2.0, 180.0)

        @JvmField
        var clipSpecimenDuration: Long = 0

        @JvmField
        var armWait = 250L

        @JvmField
        var liftWait = 250L
        // </editor-fold>

        // <editor-fold desc="Sweep Poses">
        @JvmField
        var pushX = -52.0

        @JvmField
        var slowThreshold = -38.0
        // </editor-fold>
    }
}
