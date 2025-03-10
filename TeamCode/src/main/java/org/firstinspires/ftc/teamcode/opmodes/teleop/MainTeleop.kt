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

                    val toggleAutoPickup =
                        EdgeDetector(gp2::square) {
                            toggles.autoPickup = !toggles.autoPickup
                        }
                    val toggleAssists =
                        EdgeDetector(gp2::triangle) {
                            useBasketAssist = !useBasketAssist
                            useRungAssist = !useRungAssist
                            useWallAssist = !useWallAssist
                        }

                    val liftResets =
                        EdgeDetector(
                            gp2::circle,
                            SequentialCommandGroup(
                                outtake.slides.directPower(rezeroPower),
                                outtake.slides.enableDirect(),
                            ),
                            SequentialCommandGroup(
                                outtake.slides.rezeroCmd(),
                                outtake.slides.disableDirect(),
                            ),
                        )

                    val toggleColor =
                        EdgeDetector(
                            gp2::dpad_up,
                        ) {
                            isRed = !isRed
                        }

                    val park =
                        EdgeDetector(
                            gp1::dpad_left,
                            outtake.park(),
                        )

                    fun pickupPre(
                        rumble: RunCommand,
                        useLinkage: Boolean,
                    ) = SequentialCommandGroup(
                        autoPickup.stop(),
                        intake.open(),
                        if (useLinkage) intake.extend() else intake.baseExtend(),
                        ConditionalCommand(
                            SequentialCommandGroup(
                                WaitCommand(AutoPickup.cameraStablizationDuration),
                                autoPickup.startScanning(),
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
                                        autoPickup.cancelRumble(rumble),
                                        autoPickup.stopScanning(),
                                    ),
                                    ConditionalCommand(
                                        SequentialCommandGroup(
                                            autoPickup.align(),
                                            intake.grab(),
                                            autoPickup.stop(),
                                            macros.miniTransfer(),
                                        ),
                                        intake.retract(),
                                    ) { autoPickup.lastTarget != null },
                                ),
                                SequentialCommandGroup(
                                    intake.grab(),
                                    macros.miniTransfer(),
                                ),
                            ) { toggles.autoPickup },
                        )

                    val rumble1 = autoPickup.rumbleForever()
                    val pickup =
                        EdgeDetector(
                            gp1::right_bumper,
                            pickupPre(rumble1, true),
                            pickupPost(rumble1),
                        )

                    val rumble2 = autoPickup.rumbleForever()
                    val basePickup =
                        EdgeDetector(
                            gp1::left_bumper,
                            pickupPre(rumble2, false),
                            pickupPost(rumble2),
                        )

                    // samples
                    val lowBasket =
                        EdgeDetector(
                            gp1::dpad_down,
                            SequentialCommandGroup(
                                macros.exitTransfer(),
                                outtake.slides.goTo(Slides.lowBasket),
                                outtake.basketPartial(),
                            ),
                        )

                    val highBasket =
                        EdgeDetector(
                            gp1::dpad_up,
                            SequentialCommandGroup(
                                macros.exitTransfer(),
                                outtake.slides.goTo(Slides.highBasket),
                                outtake.basketPartial(),
                            ),
                        )

                    // specimen
                    val specimenPickup =
                        EdgeDetector(
                            gp1::cross,
                            SequentialCommandGroup(
                                outtake.open(),
                                outtake.slides.goTo(Slides.min),
                                outtake.specimenPickupPartial(),
                            ),
                            SequentialCommandGroup(
                                outtake.close(),
                                WaitCommand(specimenCloseDuration),
                                ParallelCommandGroup(
                                    outtake.slides.goTo(Slides.highRung),
                                    outtake.specimenPartial(),
                                ),
                            ),
                        )

                    // scoring (sample & specimen)
                    val score =
                        EdgeDetector(
                            gp1::triangle,
                            ConditionalCommand(
                                outtake.open(),
                                outtake.slides.goTo(Slides.highRungScore),
                            ) { outtake.arm.state == OuttakeArmPosition.BASKET },
                            ConditionalCommand(
                                SequentialCommandGroup(
                                    outtake.basePartial(),
                                    outtake.base(),
                                ),
                                SequentialCommandGroup(
                                    outtake.open(),
                                    outtake.specimenPickupPartial(),
                                    outtake.slides.goTo(Slides.min),
                                    outtake.slides.reset(),
                                ),
                            ) { outtake.arm.state == OuttakeArmPosition.BASKET },
                        )

                    val imuReset =
                        EdgeDetector(
                            gamepad1::circle,
                            InstantCommand({
                                drive.pinPoint.pinPoint.setPosition(
                                    Pose2D(
                                        DistanceUnit.INCH,
                                        drive.pose.x,
                                        drive.pose.y,
                                        AngleUnit.DEGREES,
                                        0.0,
                                    ),
                                )
                                gamepad1.rumble(250)
                            }),
                        )
                }
            }
        }
    }

    override fun initialize() {
        robot.apply {
            isRed = false
            FtcDashboard.getInstance().startCameraStream(camera.sampleDetector, 0.0)
            drive.pinPoint.recalibrateImu()
        }
        triggers
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

        robot.apply {
            val attemptingToBasket =
                outtake.slides.target > (Slides.lowBasket - Slides.min) / 2 && outtake.arm.state == OuttakeArmPosition.BASKET
            val basketAssist =
                calculateAssist(useBasketAssist && attemptingToBasket, basketAssistHeading) * basketAssistWeight

            val attemptingToRung = outtake.arm.state == OuttakeArmPosition.SPECIMEN
            val rungAssist = calculateAssist(useRungAssist && attemptingToRung, rungAssistHeading) * rungAssistWeight

            val attemptingToWall = outtake.arm.state == OuttakeArmPosition.PICKUP
            val wallAssist = calculateAssist(useWallAssist && attemptingToWall, wallAssistHeading) * wallAssistWeight

            val assists = basketAssist + rungAssist + wallAssist

            if (!drive.pidManager.isOn) {
                drive.fieldCentric(
                    gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble() + assists,
                    -if (fieldCentric) drive.pose.radians else 0.0,
                )
            }

            // Slides

            val slidePower = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
            if (slidePower.absoluteValue > slideThreshold) {
                outtake.slides.isManual = true
                if (intake.claw.state != IntakeClawState.OPEN) {
                    outtake.slides.manualPower = slidePower.clamp(-1.0, 0)
                } else {
                    outtake.slides.manualPower = slidePower
                }
            } else {
                outtake.slides.isManual = false
            }

            // we can make triggers do either rotate or slides based on the arm states?

            telemetry.addData("pid | on", drive.pidManager.isOn)
            telemetry.addData("isRed", isRed)
            telemetry.addData("pid | pid target", drive.pidManager.target)
            telemetry.addData("slides", outtake.slides.position)
            telemetry.addData("toggles | autopickup", toggles.autoPickup)
            telemetry.addData("assists | basket assist", useBasketAssist)
            telemetry.addData("assists | rung assist", useRungAssist)
            telemetry.addData("assists | wall assist", useWallAssist)
        }
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
