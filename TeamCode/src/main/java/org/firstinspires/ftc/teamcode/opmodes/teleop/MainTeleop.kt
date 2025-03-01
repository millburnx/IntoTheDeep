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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.AutoPickup
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides.Companion.rezeroPower
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.normalizeDegrees
import org.firstinspires.ftc.teamcode.opmodes.auton.SpecimenAuton.Companion.scoringDuration
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleCameraRobot
import kotlin.math.absoluteValue

class TeleOpToggles {
    var autoPickup = true
    var useAlternateSpec = false
}

@Config
@TeleOp(name = "Main Teleop")
class MainTeleop : OpMode() {
    override val robot: SampleCameraRobot by lazy { SampleCameraRobot(this) }

    val toggles = TeleOpToggles()

    val triggers by lazy {
        object {
            val toggleAutoPickup =
                EdgeDetector(gamepad2::square) {
                    toggles.autoPickup = !toggles.autoPickup
                }
            val toggleAssists =
                EdgeDetector(gamepad2::triangle) {
                    useBasketAssist = !useBasketAssist
                    useRungAssist = !useRungAssist
                    useWallAssist = !useWallAssist
                }

            val liftResets =
                EdgeDetector(
                    gamepad2::circle,
                    this@MainTeleop,
                    SequentialCommandGroup(
                        robot.outtake.slides.directPower(rezeroPower),
                        robot.outtake.slides.enableDirect(),
                    ),
                    SequentialCommandGroup(
                        robot.outtake.slides.rezeroCmd(),
                        robot.outtake.slides.disableDirect(),
                    ),
                )

            val alternativeSpec = EdgeDetector(gamepad2::cross) { toggles.useAlternateSpec = !toggles.useAlternateSpec }

            fun pickupPre(
                rumble: RunCommand,
                useLinkage: Boolean,
            ) = SequentialCommandGroup(
                robot.autoPickup.stop(),
                robot.macros.exitSpecPickup(),
                robot.intake.open(),
                if (useLinkage) robot.intake.extend() else robot.intake.baseExtend(),
                ConditionalCommand(
                    SequentialCommandGroup(
                        WaitCommand(AutoPickup.cameraStablizationDuration),
                        robot.autoPickup.startScanning(),
                        rumble,
                    ),
                    InstantCommand({}),
                ) { toggles.autoPickup },
            )

            fun pickupPost(rumble: RunCommand) =
                ConditionalCommand(
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            robot.autoPickup.cancelRumble(rumble),
                            robot.autoPickup.stopScanning(),
                        ),
                        ConditionalCommand(
                            SequentialCommandGroup(
                                robot.autoPickup.align(),
                                robot.intake.grab(),
                                robot.autoPickup.stop(),
                                robot.macros.transfer(),
                            ),
                            robot.intake.retract(),
                        ) { robot.autoPickup.lastTarget != null },
                    ),
                    SequentialCommandGroup(
                        robot.intake.grab(),
                        robot.macros.transfer(),
                    ),
                ) { toggles.autoPickup }

            val rumble1 = robot.autoPickup.rumbleForever()
            val pickup =
                EdgeDetector(
                    robot.gp1::right_bumper,
                    this@MainTeleop,
                    pickupPre(rumble1, true),
                    pickupPost(rumble1),
                )

            val rumble2 = robot.autoPickup.rumbleForever()
            val basePickup =
                EdgeDetector(
                    robot.gp1::left_bumper,
                    this@MainTeleop,
                    pickupPre(rumble2, false),
                    pickupPost(rumble2),
                )

            // samples
            val lowBasket =
                EdgeDetector(
                    robot.gp1::dpad_down,
                    this@MainTeleop,
                    SequentialCommandGroup(
                        SlidesCommand(robot.outtake.slides, Slides.lowBasket),
                        robot.outtake.arm.basket(),
                        robot.outtake.wrist.basket(),
                    ),
                )

            val highBasket =
                EdgeDetector(
                    robot.gp1::dpad_up,
                    this@MainTeleop,
                    SequentialCommandGroup(
                        SlidesCommand(robot.outtake.slides, Slides.highBasket),
                        robot.outtake.arm.basket(),
                        robot.outtake.wrist.basket(),
                    ),
                )

            // specimen
            val specimenPickup =
                EdgeDetector(
                    robot.gp1::cross,
                    this@MainTeleop,
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
                            ) { robot.intake.arm.state != IntakeArmPosition.SPECIMEN },
                        ),
                        ParallelCommandGroup(
                            robot.outtake.arm.pickup(),
                            robot.outtake.wrist.pickup(),
                        ),
                        robot.outtake.open(),
                    ),
                    SequentialCommandGroup(
                        robot.outtake.close(),
                        WaitCommand(specimenCloseDuration),
                        ConditionalCommand(
                            ParallelCommandGroup(
                                SlidesCommand(robot.outtake.slides, Slides.wall),
                                robot.outtake.arm.specimen(),
                                robot.outtake.wrist.specimen(),
                            ),
                            ParallelCommandGroup(
                                SlidesCommand(robot.outtake.slides, Slides.highRung),
                                robot.outtake.arm.altSpecimen(),
                                robot.outtake.wrist.altSpecimen(),
                            ),
                        ) { !toggles.useAlternateSpec },
                    ),
                )

            val hpDrop =
                EdgeDetector(
                    robot.gp1::square,
                    this@MainTeleop,
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            SlidesCommand(robot.outtake.slides, Slides.min),
                            robot.outtake.arm.human(),
                            robot.outtake.wrist.human(),
                        ),
                        robot.outtake.open(),
                    ),
                )

            // scoring (sample & specimen)
            val score =
                EdgeDetector(
                    robot.gp1::triangle,
                    this@MainTeleop,
                    ConditionalCommand(
                        SequentialCommandGroup(
                            robot.outtake.open(),
                        ),
                        ConditionalCommand(
                            SequentialCommandGroup(
                                robot.outtake.arm.specimenScoring(),
                                WaitCommand(scoringDuration),
                                SlidesCommand(robot.outtake.slides, Slides.wall2),
                            ),
                            SequentialCommandGroup(
                                SlidesCommand(robot.outtake.slides, Slides.highRungScore),
                            ),
                        ) { !toggles.useAlternateSpec },
                    ) { robot.outtake.arm.state == OuttakeArmPosition.BASKET },
                    ConditionalCommand(
                        SequentialCommandGroup(
                            robot.outtake.wrist.base(),
                            robot.outtake.arm.base(),
                            robot.outtake.base(),
                        ),
                        SequentialCommandGroup(
                            robot.outtake.open(),
                            ParallelCommandGroup(
                                ConditionalCommand(
                                    ParallelCommandGroup(
                                        robot.intake.arm.specimen(),
                                        robot.intake.diffy.specimen(),
                                        WaitCommand(intakeLoweringDuration),
                                    ),
                                    InstantCommand({}),
                                ) { robot.intake.arm.state != IntakeArmPosition.SPECIMEN },
                            ),
                            ParallelCommandGroup(
                                robot.outtake.arm.pickup(),
                                robot.outtake.wrist.pickup(),
                            ),
                            SlidesCommand(robot.outtake.slides, Slides.min),
                            robot.outtake.slides.reset(),
                        ),
                    ) { robot.outtake.arm.state == OuttakeArmPosition.BASKET },
                )

            val imuReset =
                EdgeDetector(
                    gamepad1::circle,
                    this@MainTeleop,
                    InstantCommand({
                        robot.imu.resetYaw()
                        gamepad1.rumble(250)
                    }),
                )
        }
    }

    override fun initialize() {
        FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector, 0.0)
        triggers
    }

    var hasInit = false

    fun calculateAssist(
        isAttempting: Boolean,
        targetAngle: Double,
    ): Double {
        if (!isAttempting) return 0.0
        val currentAngle = robot.imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
        val diff = normalizeDegrees(targetAngle - currentAngle)
        return diff
    }

    override fun exec() {
        if (!hasInit) {
            super.initialize()
            hasInit = true
        }

        val attemptingToBasket =
            robot.outtake.slides.target > (Slides.lowBasket - Slides.min) / 2 && robot.outtake.arm.state == OuttakeArmPosition.BASKET
        val basketAssist =
            calculateAssist(useBasketAssist && attemptingToBasket, basketAssistHeading) * basketAssistWeight

        val attemptingToRung = robot.outtake.arm.state == OuttakeArmPosition.SPECIMEN
        val rungAssist = calculateAssist(useRungAssist && attemptingToRung, rungAssistHeading) * rungAssistWeight

        val attemptingToWall = robot.outtake.arm.state == OuttakeArmPosition.PICKUP
        val wallAssist = calculateAssist(useWallAssist && attemptingToWall, wallAssistHeading) * wallAssistWeight

        val assists = basketAssist + rungAssist + wallAssist

        if (!robot.drive.pidManager.isOn) {
            robot.drive.fieldCentric(
                gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble() + assists,
                if (fieldCentric) -robot.imuHeading(AngleUnit.RADIANS) else 0.0,
            )
        }

        // Slides
        val slidePower = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
        if (slidePower.absoluteValue > slideThreshold) {
            robot.outtake.slides.isManual = true
            if (!robot.intake.claw.isOpen) {
                robot.outtake.slides.manualPower = slidePower.clamp(-1.0, 0)
            } else {
                robot.outtake.slides.manualPower = slidePower
            }
        } else {
            robot.outtake.slides.isManual = false
        }

        robot.telemetry.addData("pid | on", robot.drive.pidManager.isOn)
        robot.telemetry.addData("pid | pid target", robot.drive.pidManager.target)
        robot.telemetry.addData("hz | Delta Time", robot.deltaTime.deltaTime)
        robot.telemetry.addData("hz | Loop Hertz", 1.0 / robot.deltaTime.deltaTime)
        robot.telemetry.addData("slides", robot.outtake.slides.position)
        robot.telemetry.addData("toggles | autopickup", toggles.autoPickup)
        robot.telemetry.addData("toggles | alt spec", toggles.useAlternateSpec)
        robot.telemetry.addData("assists | basket assist", useBasketAssist)
        robot.telemetry.addData("assists | rung assist", useRungAssist)
        robot.telemetry.addData("assists | wall assist", useWallAssist)
    }

    companion object {
        @JvmField
        var isRed: Boolean = false

        @JvmField
        var fieldCentric: Boolean = true

        @JvmField
        var slideThreshold: Double = 0.1

        // delays

        @JvmField
        var transferArmDelay: Long = 250

        @JvmField
        var transferClawDelay: Long = 250

        @JvmField
        var outtakeFlipDelay: Long = 250

        @JvmField
        var intakeLoweringDuration: Long = 750

        @JvmField
        var outtakeLiftingDuration: Long = 500

        @JvmField
        var specimenCloseDuration: Long = 250

        @JvmField
        var outtakeDropArmDelay: Long = 250

        @JvmField
        var intakePickupArmDelay: Long = 500

        @JvmField
        var intakePickupClawDelay: Long = 250

        @JvmField
        var baseIntakeDuration: Long = 500

        @JvmField
        var intakeDuration: Long = 625

        // Heading assist

        @JvmField
        var useBasketAssist: Boolean = true

        @JvmField
        var basketAssistWeight: Double = 0.025

        @JvmField
        var basketAssistHeading: Double = -45.0

        @JvmField
        var useRungAssist: Boolean = true

        @JvmField
        var rungAssistWeight: Double = 0.025

        @JvmField
        var rungAssistHeading: Double = 180.0

        @JvmField
        var useWallAssist: Boolean = true

        @JvmField
        var wallAssistWeight: Double = 0.025

        @JvmField
        var wallAssistHeading: Double = 180.0
    }
}
