package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.AutoPickup
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.reset
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleCameraRobot
import kotlin.math.absoluteValue

@Config
@TeleOp(name = "Basic Teleop")
class BasicTeleop : OpMode() {
    override val robot: SampleCameraRobot by lazy { SampleCameraRobot(this) }

    var extensionTimer: ElapsedTime? = null

    val triggers by lazy {
        object {
            val linkagePickup =
                EdgeDetector(
                    gamepad1::right_bumper,
                    this@BasicTeleop,
                    SequentialCommandGroup(
                        InstantCommand({ robot.pidManager.isOn = false }),
                        ParallelCommandGroup(
                            robot.intake.extend(),
                            robot.intake.open(),
                            robot.outtake.base(),
//                            InstantCommand({
//                                extensionTimer = ElapsedTime()
//                            }),
                        ),
//                        WaitUntilCommand({ extensionTimer!!.milliseconds() > intakeDuration }),
                        WaitCommand(250L),
                        InstantCommand({ robot.autoPickup.scanning = true }),
                        RunCommand({
                            if (robot.autoPickup.lastTarget != null) {
                                gamepad1.rumble(
                                    (AutoPickup.cacheDuration - 200)
                                        .coerceAtLeast(robot.deltaTime.deltaTime * 1000)
                                        .toInt(),
                                )
                            }
                        }, robot.intake),
                    ),
                    SequentialCommandGroup(
                        InstantCommand({ robot.autoPickup.scanning = false }),
                        ConditionalCommand(
                            SequentialCommandGroup(
                                robot.autoPickup.align(),
                                robot.intake.grab(),
                                InstantCommand({ robot.pidManager.isOn = false }),
                                ParallelCommandGroup(
                                    robot.intake.retract(),
                                    robot.outtake.open(),
                                    robot.outtake.base(),
                                ),
                                robot.outtake.close(),
                                WaitCommand(transferClawDelay),
                                robot.intake.open(),
                            ),
//                            SequentialCommandGroup(
//                                ParallelCommandGroup(
//                                    robot.intake.retract(),
//                                    robot.outtake.base(),
//                                ),
//                            ),
                            robot.intake.retract(),
                            { robot.autoPickup.lastTarget != null },
                        ),
                    ),
                )

            val specimenPickup =
                EdgeDetector(gamepad1::circle, {
                    val atBase = robot.outtake.arm.state == OuttakeArmPosition.BASE

                    schedule(
                        SequentialCommandGroup(
                            InstantCommand({ robot.outtake.claw.isOpen = true }, robot.outtake),
                            InstantCommand({
                                robot.intake.arm.state = IntakeArmPosition.SPECIMEN
                                robot.intake.diffy.roll = Diffy.specimenRoll
                                robot.intake.diffy.pitch = Diffy.specimenPitch
                            }, robot.intake),
                            WaitCommand(specimenDelay),
                            InstantCommand({
                                robot.outtake.arm.state = OuttakeArmPosition.OUT
                                robot.outtake.wrist.state = OuttakeWristPosition.OUT
                            }, robot.outtake),
                        ),
                    )
                }, {
                    schedule(
                        SequentialCommandGroup(
                            InstantCommand({ robot.outtake.claw.isOpen = false }, robot.outtake),
                            WaitCommand(outtakePickupClawDelay),
                            ParallelCommandGroup(
                                InstantCommand({
                                    robot.outtake.arm.state = OuttakeArmPosition.SPECIMEN
                                    robot.outtake.wrist.state = OuttakeWristPosition.SPECIMEN
                                    robot.intake.arm.state = IntakeArmPosition.BASE
                                    robot.intake.diffy.roll = Diffy.transferRoll
                                    robot.intake.diffy.pitch = Diffy.transferPitch
                                }, robot.outtake),
                                SlidesCommand(robot.outtake.slides, Slides.highRung),
                            ),
                        ),
                    )
                })

            val outtakeClaw =
                EdgeDetector(gamepad1::triangle, {
                    // prevent dropping in bot
                    if (robot.outtake.arm.state == OuttakeArmPosition.BASE) return@EdgeDetector
                    if (robot.outtake.claw.isOpen) {
                        // stop slides and close claw (miss protection)
                        schedule(
                            ParallelCommandGroup(
                                InstantCommand(robot.outtake.claw::close, robot.outtake.claw),
                                SlidesCommand(robot.outtake.slides, robot.outtake.slides.position),
                            ),
                        )
                    } else {
                        robot.outtake.claw.open()
                        schedule(
                            SequentialCommandGroup(
                                InstantCommand(robot.outtake.claw::open, robot.outtake.claw),
                            ),
                        )
                    }
                }, {
                    // go to base
                    if (!robot.outtake.claw.isOpen) return@EdgeDetector
                    schedule(
                        SequentialCommandGroup(
                            ParallelCommandGroup(
                                InstantCommand({
                                    robot.outtake.arm.state = OuttakeArmPosition.BASE
                                    robot.outtake.wrist.state = OuttakeWristPosition.BASE
                                }, robot.outtake.arm, robot.outtake.wrist),
                                WaitCommand(outtakeDropArmDelay),
                            ),
                            SlidesCommand(robot.outtake.slides, Slides.min),
                        ),
                    )
                })

//            val specimenScore =
//                EdgeDetector(
//                    gamepad1::dpad_left,
//                    this@BasicTeleop,
//                    ParallelCommandGroup(
//                        InstantCommand({
//                            robot.outtake.arm.state = OuttakeArmPosition.SPECIMEN
//                            robot.outtake.wrist.state = OuttakeWristPosition.SPECIMEN
//                        }, robot.outtake),
//                        SlidesCommand(robot.outtake.slides, Slides.highRung),
//                    ),
//                )

            val highSampleScore =
                EdgeDetector(
                    gamepad1::dpad_up,
                ) {
                    if (!robot.intake.claw.isOpen) return@EdgeDetector
                    schedule(
                        SequentialCommandGroup(
                            InstantCommand({
                                robot.outtake.arm.state = OuttakeArmPosition.BASKET
                                robot.outtake.wrist.state = OuttakeWristPosition.BASKET
                            }, robot.outtake),
                            ParallelCommandGroup(
                                InstantCommand({
                                    robot.intake.linkage.target = 0.0
                                    robot.intake.arm.state = IntakeArmPosition.BASE
                                    robot.intake.diffy.pitch = Diffy.transferPitch
                                    robot.intake.diffy.roll = Diffy.transferRoll
                                }, robot.intake),
                                SlidesCommand(robot.outtake.slides, Slides.highBasket),
                            ),
                        ),
                    )
                }

            val lowSampleScore =
                EdgeDetector(
                    gamepad1::dpad_right,
                ) {
                    if (!robot.intake.claw.isOpen) return@EdgeDetector
                    schedule(
                        SequentialCommandGroup(
                            InstantCommand({
                                robot.outtake.arm.state = OuttakeArmPosition.BASKET
                                robot.outtake.wrist.state = OuttakeWristPosition.BASKET
                            }, robot.outtake),
                            ParallelCommandGroup(
                                InstantCommand({
                                    robot.intake.linkage.target = 0.0
                                    robot.intake.arm.state = IntakeArmPosition.BASE
                                    robot.intake.diffy.pitch = Diffy.transferPitch
                                    robot.intake.diffy.roll = Diffy.transferRoll
                                }, robot.intake),
                                SlidesCommand(robot.outtake.slides, Slides.lowBasket),
                            ),
                        ),
                    )
                }

            val humanDrop =
                EdgeDetector(
                    gamepad1::dpad_down,
                    this@BasicTeleop,
                    InstantCommand({
                        robot.outtake.arm.state = OuttakeArmPosition.HUMAN
                        robot.outtake.wrist.state = OuttakeWristPosition.HUMAN
                    }),
                )

//            val imuReset = EdgeDetector(
//                gamepad2::b,
//                this@BasicTeleop,
//                InstantCommand({
//                    robot.imu.resetYaw()
//                    gamepad2.rumble(250)
//                })
//            )
//
//            val liftReset = EdgeDetector(
//                gamepad2::dpad_down,
//                {
//                    gamepad2.rumble(250)
//                },
//                {
//                    robot.outtake.slides.isManual = false
//                    robot.outtake.slides.leftLift.reset()
//                    robot.outtake.slides.rightLift.reset()
//                    gamepad2.rumble(250)
//                }
//            )

            val reset =
                EdgeDetector(
                    gamepad1::cross,
                    {
                        robot.imu.resetYaw()
                        gamepad1.rumble(250)
                    },
                    {
                        robot.outtake.slides.isManual = false
                        robot.outtake.slides.leftLift
                            .reset()
                        robot.outtake.slides.rightLift
                            .reset()
                        gamepad2.rumble(250)
                    },
                )

            fun instCmd(cmd: Pair<() -> Unit, List<Subsystem>>): InstantCommand = InstantCommand(cmd.first, *cmd.second.toTypedArray())
        }
    }

    override fun initialize() {
        FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector, 0.0)
        triggers
    }

    var hasInit = false

    override fun exec() {
        if (!hasInit) {
            super.initialize()
            hasInit = true
        }

        if (fieldCentric) {
            robot.drive.fieldCentric(
                gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                Math.toRadians(-robot.imu.robotYawPitchRollAngles.yaw),
            )
        } else {
            robot.drive.robotCentric(
                gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
            )
        }

        // Slides
        val slidePower = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
        if (gamepad1.cross) {
            robot.outtake.slides.isManual = true
            robot.outtake.slides.manualPower = -1.0
        } else if (slidePower.absoluteValue > slideThreshold) {
            robot.outtake.slides.isManual = true
            if (!robot.intake.claw.isOpen) {
                robot.outtake.slides.manualPower = slidePower.clamp(-1.0, 0)
            } else {
                robot.outtake.slides.manualPower = slidePower
            }
        } else {
            robot.outtake.slides.isManual = false
        }

//        robot.telemetryManager.send()

        robot.telemetry.addData("pid", robot.pidManager.isOn)
        robot.telemetry.addData("pid target", robot.pidManager.target)
        robot.telemetry.addData("Delta Time", robot.deltaTime.deltaTime)
        robot.telemetry.addData("Loop Hertz", 1.0 / robot.deltaTime.deltaTime)
    }

    companion object {
        @JvmField
        var isRed: Boolean = false

        @JvmField
        var slideThreshold: Double = 0.1

        @JvmField
        var intakePickupArmDelay: Long = 250

        @JvmField
        var intakePickupClawDelay: Long = 250

        @JvmField
        var pickupTimeout: Long = 750

        @JvmField
        var transferClawDelay: Long = 200

        @JvmField
        var outtakePickupClawDelay: Long = 250

        @JvmField
        var intakeDuration: Long = 1000

        @JvmField
        var fieldCentric: Boolean = true

        @JvmField
        var outtakeDropArmDelay: Long = 250

        @JvmField
        var specimenDelay: Long = 500

        @JvmField
        var specScoreDelay: Long = 625

        @JvmField
        var sweepDuration: Long = 1500
    }
}
