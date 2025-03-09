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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.common.subsystems.drive.AutoPickup
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeClawState
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides.Companion.rezeroPower
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.normalizeDegrees
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleCameraRobot
import kotlin.math.absoluteValue

class TeleOpToggles {
    var autoPickup = true
}

@Config
@TeleOp(name = "Main Teleop Blue")
open class MainTeleopBlue : OpMode() {
    override val robot: SampleCameraRobot by lazy { SampleCameraRobot(this) }

    val toggles = TeleOpToggles()

    val triggers by lazy {
        object {
            init {
                robot.apply {
                    // we can move shit here so you don't have to spam robot.foo
                    // only downside is you not longer can directly access triggers in the rest of the opmode
                    // unless we make triggers a property of robot
                    // or placing all the triggers in a hashmap (both have poor typing)
                }
            }

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

            val park =
                EdgeDetector(
                    gamepad1::dpad_left,
                    robot.outtake.park(),
                )

            val liftResets =
                EdgeDetector(
                    gamepad2::circle,
                    SequentialCommandGroup(
                        robot.outtake.slides.directPower(rezeroPower),
                        robot.outtake.slides.enableDirect(),
                    ),
                    SequentialCommandGroup(
                        robot.outtake.slides.rezeroCmd(),
                        robot.outtake.slides.disableDirect(),
                    ),
                )

            val toggleColor =
                EdgeDetector(
                    gamepad2::dpad_up,
                ) {
                    robot.isRed = !robot.isRed
                }

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
                SequentialCommandGroup(
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
                                    robot.macros.miniTransfer(),
                                ),
                                robot.intake.retract(),
                            ) { robot.autoPickup.lastTarget != null },
                        ),
                        SequentialCommandGroup(
                            robot.intake.grab(),
                            robot.macros.miniTransfer(),
                        ),
                    ) { toggles.autoPickup },
                )

            val rumble1 = robot.autoPickup.rumbleForever()
            val pickup =
                EdgeDetector(
                    robot.gp1::right_bumper,
                    pickupPre(rumble1, true),
                    pickupPost(rumble1),
                )

            val rumble2 = robot.autoPickup.rumbleForever()
            val basePickup =
                EdgeDetector(
                    robot.gp1::left_bumper,
                    pickupPre(rumble2, false),
                    pickupPost(rumble2),
                )

            // samples
            val lowBasket =
                EdgeDetector(
                    robot.gp1::dpad_down,
                    SequentialCommandGroup(
                        robot.macros.exitTransfer(),
                        robot.outtake.slides.goTo(Slides.lowBasket),
                        robot.outtake.basketPartial(),
                    ),
                )

            val highBasket =
                EdgeDetector(
                    robot.gp1::dpad_up,
                    SequentialCommandGroup(
                        robot.macros.exitTransfer(),
                        robot.outtake.slides.goTo(Slides.highBasket),
                        robot.outtake.basketPartial(),
                    ),
                )

            // specimen
            val specimenPickup =
                EdgeDetector(
                    robot.gp1::cross,
                    SequentialCommandGroup(
                        robot.outtake.open(),
                        robot.outtake.slides.goTo(Slides.min),
                        robot.outtake.specimenPickupPartial(),
                    ),
                    SequentialCommandGroup(
                        robot.outtake.close(),
                        WaitCommand(specimenCloseDuration),
                        ParallelCommandGroup(
                            robot.outtake.slides.goTo(Slides.highRung),
                            robot.outtake.specimenPartial(),
                        ),
                    ),
                )

            // scoring (sample & specimen)
            val score =
                EdgeDetector(
                    robot.gp1::triangle,
                    ConditionalCommand(
                        robot.outtake.open(),
                        robot.outtake.slides.goTo(Slides.highRungScore),
                    ) { robot.outtake.arm.state == OuttakeArmPosition.BASKET },
                    ConditionalCommand(
                        SequentialCommandGroup(
                            robot.outtake.basePartial(),
                            robot.outtake.base(),
                        ),
                        SequentialCommandGroup(
                            robot.outtake.open(),
                            robot.outtake.specimenPickupPartial(),
                            robot.outtake.slides.goTo(Slides.min),
                            robot.outtake.slides.reset(),
                        ),
                    ) { robot.outtake.arm.state == OuttakeArmPosition.BASKET },
                )

            val imuReset =
                EdgeDetector(
                    gamepad1::circle,
                    InstantCommand({
//                        robot.drive.pinPoint.resetImu()
                        robot.drive.pinPoint.pinPoint.setPosition(
                            Pose2D(
                                DistanceUnit.INCH,
                                robot.drive.pose.x,
                                robot.drive.pose.y,
                                AngleUnit.DEGREES,
                                0.0,
                            ),
                        )
                        gamepad1.rumble(250)
                    }),
                )
        }
    }

    override fun initialize() {
        robot.isRed = false
        FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector, 0.0)
        triggers
        robot.drive.pinPoint.recalibrateImu()
    }

    var hasInit = false

    fun calculateAssist(
        isAttempting: Boolean,
        targetAngle: Double,
    ): Double {
        if (!isAttempting) return 0.0
        val currentAngle = robot.drive.pose.heading
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
                -if (fieldCentric) robot.drive.pose.radians else 0.0,
            )
        }

        // Slides

        val slidePower = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
        if (slidePower.absoluteValue > slideThreshold) {
            robot.outtake.slides.isManual = true
            if (robot.intake.claw.state != IntakeClawState.OPEN) {
                robot.outtake.slides.manualPower = slidePower.clamp(-1.0, 0)
            } else {
                robot.outtake.slides.manualPower = slidePower
            }
        } else {
            robot.outtake.slides.isManual = false
        }

        // we can make triggers do either rotate or slides based on the arm states?

        robot.telemetry.addData("pid | on", robot.drive.pidManager.isOn)
        robot.telemetry.addData("isRed", robot.isRed)
        robot.telemetry.addData("pid | pid target", robot.drive.pidManager.target)
        robot.telemetry.addData("hz | Delta Time", robot.deltaTime.deltaTime)
        robot.telemetry.addData("hz | Loop Hertz", 1.0 / robot.deltaTime.deltaTime)
        robot.telemetry.addData("slides", robot.outtake.slides.position)
        robot.telemetry.addData("toggles | autopickup", toggles.autoPickup)
        robot.telemetry.addData("assists | basket assist", useBasketAssist)
        robot.telemetry.addData("assists | rung assist", useRungAssist)
        robot.telemetry.addData("assists | wall assist", useWallAssist)
    }

    companion object {
        @JvmField
        var fieldCentric: Boolean = true

        @JvmField
        var slideThreshold: Double = 0.1

        @JvmField
        var diffyMultipler: Double = 0.2

        // delays

        @JvmField
        var transferArmDelay: Long = 250

        @JvmField
        var transferClawDelay: Long = 250

        @JvmField
        var outtakeFlipDelay: Long = 500

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
        var baseIntakeDuration: Long = 1250

        @JvmField
        var intakeDuration: Long = 1500

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
        var rungAssistHeading: Double = 0.0

        @JvmField
        var useWallAssist: Boolean = true

        @JvmField
        var wallAssistWeight: Double = 0.025

        @JvmField
        var wallAssistHeading: Double = 0.0
    }
}
