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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.common.subsystems.drive.AutoPickup
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeClaw
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeClaw
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.conditionalCommand
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
                        EdgeDetector(button = gp2::square) {
                            toggles.autoPickup = !toggles.autoPickup
                        }

                    val leftShiftDown =
                        EdgeDetector(button = gp2::dpad_left) {
                            intake.diffy.left.rotations -= 1
                        }
                    val leftShiftUp =
                        EdgeDetector(button = gp2::left_bumper) {
                            intake.diffy.left.rotations += 1
                        }
                    val rightShiftDown =
                        EdgeDetector(button = gp2::dpad_right) {
                            intake.diffy.right.rotations -= 1
                        }
                    val rightShiftUp =
                        EdgeDetector(button = gp2::right_bumper) {
                            intake.diffy.right.rotations += 1
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
                            outtake.slides.goTo(Slides.State.REZERO),
                            SequentialCommandGroup(
                                outtake.slides.rezeroCmd(),
                                outtake.slides.goTo(Slides.State.BASE),
                            ),
                        )

                    val toggleYellow =
                        EdgeDetector(
                            gp2::cross,
                        ) {
                            robot.doYellow = !robot.doYellow
                        }

                    val toggleColor =
                        EdgeDetector(
                            gp2::dpad_up,
                        ) {
                            isRed = !isRed
                        }

                    val dropToHuman =
                        EdgeDetector(
                            gp1::square,
                            SequentialCommandGroup(
                                InstantCommand({ drive.pidManager.isOn = false }),
                                intake.close(),
                                conditionalCommand(
                                    WaitCommand(transferClawDelay)
                                        .andThen(
                                            outtake.open(),
                                            WaitCommand(transferClawDelay),
                                        ),
                                ) { outtake.claw.state != OuttakeClaw.State.OPEN },
                                conditionalCommand(
                                    outtake.basePartial().andThen(
                                        WaitCommand(transferClawDelay),
                                    ),
                                ) {
                                    outtake.arm.state == OuttakeArmPosition.TRANSFER
                                },
                                intake.extend(),
                            ),
                            SequentialCommandGroup(
                                intake.open(),
                                intake.retract(),
                            ),
                        )

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
                        conditionalCommand(
                            outtake.basePartial().andThen(
                                WaitCommand(transferClawDelay),
                            ),
                        ) {
                            outtake.arm.state == OuttakeArmPosition.TRANSFER
                        },
                        conditionalCommand(
                            outtake.basePartial(),
                        ) {
                            outtake.arm.state != OuttakeArmPosition.BASE
                        },
                        if (useLinkage) {
                            ConditionalCommand(
                                intake.extend(),
                                intake.extendLow(),
                            ) { toggles.autoPickup }
                        } else {
                            ConditionalCommand(
                                intake.baseExtend(),
                                intake.baseExtendLow(),
                            ) { toggles.autoPickup }
                        },
                        ConditionalCommand(
                            WaitCommand(AutoPickup.cameraStablizationDuration).andThen(
                                autoPickup.startScanning(),
                                rumble,
                            ),
                            InstantCommand({ robot.intake.diffy.state = Diffy.State.PICKUP }),
                        ) { toggles.autoPickup },
                    )

                    fun pickupPost(rumble: RunCommand) =
                        SequentialCommandGroup(
                            ConditionalCommand(
                                SequentialCommandGroup(
                                    autoPickup.cancelRumble(rumble).alongWith(
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
                                outtake.slides.goTo(Slides.State.LOW_BASKET),
                                outtake.basketPartial(),
                            ),
                        )

                    val highBasket =
                        EdgeDetector(
                            gp1::dpad_up,
                            SequentialCommandGroup(
                                macros.exitTransfer(),
                                outtake.slides.goTo(Slides.State.HIGH_BASKET),
                                outtake.basketPartial(),
                            ),
                        )

                    // specimen
                    val specimenPickup =
                        EdgeDetector(
                            gp1::cross,
                            SequentialCommandGroup(
                                outtake.open(),
                                outtake.slides.goTo(Slides.State.BASE),
                                outtake.specimenPickupPartial(),
                            ),
                            SequentialCommandGroup(
                                outtake.close(),
                                WaitCommand(specimenCloseDuration).andThen(
                                    ParallelCommandGroup(
                                        outtake.slides.goTo(Slides.State.HIGH_RUNG),
                                        outtake.specimenPartial(),
                                    ),
                                ),
                            ),
                        )

                    // scoring (sample & specimen)
                    val score =
                        EdgeDetector(
                            gp1::triangle,
                            ConditionalCommand(
                                outtake.open(),
                                outtake.slides.goTo(Slides.State.HIGH_RUNG_SCORE),
                            ) { outtake.arm.state == OuttakeArmPosition.BASKET },
                            ConditionalCommand(
                                outtake.base(),
                                SequentialCommandGroup(
                                    outtake.open(),
                                    outtake.specimenPickupPartial(),
                                    outtake.slides.goTo(Slides.State.BASE),
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
            telemetry.addData("left", Diffy.lastLeftRotations)
            telemetry.addData("right", Diffy.lastRightRotations)
            telemetry.update()
            intake.diffy.restoreDiffyRotations()
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

    var teleopHoldTimer: ElapsedTime? = ElapsedTime()

    override fun exec() {
        if (!hasInit) {
            super.initialize()
            hasInit = true
        }

        robot.apply {
            useTeleopHold = Companion.useTeleopHold
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
                val forward = gp1.left_stick_y.toDouble()
                val strafe = -gp1.left_stick_x.toDouble()
                val rotate = -gp1.right_stick_x.toDouble()

                val driveControl = {
                    drive.pidManager.isTeleopHolding = false
                    val rotationalSpeed = if (intake.linkage.target == 1.0) linkageRotationMultiplier else 1.0

                    drive.fieldCentric(
                        forward,
                        strafe,
                        rotate * rotationalSpeed + assists,
                        -if (fieldCentric) drive.pose.radians else 0.0,
                    )
                }

                fun getLockHeading(default: Double): Double =
                    if (useBasketAssist && attemptingToBasket) {
                        basketAssistHeading
                    } else if (useWallAssist && attemptingToWall) {
                        wallAssistHeading
                    } else if (useRungAssist && attemptingToRung) {
                        rungAssistHeading
                    } else {
                        default
                    }

                if (Companion.useTeleopHold &&
                    forward.absoluteValue < minJoystickValue &&
                    strafe.absoluteValue < minJoystickValue &&
                    rotate.absoluteValue < minJoystickValue
                ) {
                    if ((teleopHoldTimer?.milliseconds() ?: 0.0) > teleopHoldDuration) {
                        val target = drive.pose
                        drive.pidManager.target = Pose2d(target.x, target.y, getLockHeading(target.heading))
                        drive.pidManager.isTeleopHolding = true
                        teleopHoldTimer = null // we only want to run this when the timer just
                    } else if (teleopHoldTimer != null) {
                        driveControl()
                    } else {
                        val target = drive.pidManager.target
                        drive.pidManager.target = Pose2d(target.x, target.y, getLockHeading(target.heading))
                    }
                } else {
                    if (teleopHoldTimer == null) {
                        teleopHoldTimer = ElapsedTime()
                    } else {
                        teleopHoldTimer?.reset()
                    }
                    driveControl()
                }
            }

            // Slides

            val slidePower = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
            if (slidePower.absoluteValue > slideThreshold) {
                outtake.slides.state = Slides.State.DIRECT
                if (intake.claw.state != IntakeClaw.State.OPEN) {
                    outtake.slides.power = slidePower.clamp(-1.0, 0)
                } else {
                    outtake.slides.power = slidePower
                }
            } else {
                if (outtake.slides.state == Slides.State.DIRECT) {
                    outtake.slides.state = Slides.State.MANUAL
                    outtake.slides.target = outtake.slides.position
                }
            }

            if (robot.intake.diffy.state == Diffy.State.PICKUP && toggles.autoPickup == false) {
                intake.diffy.roll = Diffy.hoverRoll + gp2.right_stick_x * diffyPower
            }

            // we can make triggers do either rotate or slides based on the arm states?

//            telemetry.addData("pid | on", drive.pidManager.isOn)
//            telemetry.addData("pid | hold", drive.pidManager.isTeleopHolding)
            telemetry.addData("timer", teleopHoldTimer?.milliseconds() ?: -1.0)
            telemetry.addData("isRed", isRed)
            telemetry.addData("yellow", doYellow)
            telemetry.addData("left", Diffy.lastLeftRotations)
            telemetry.addData("right", Diffy.lastRightRotations)
//            telemetry.addData("pid | pid target", drive.pidManager.target)
//            telemetry.addData("slides", outtake.slides.position)
//            telemetry.addData("slides target", outtake.slides.actualTarget)
//            telemetry.addData("slides state", outtake.slides.state)
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
        var slideThreshold: Double = 0.25

        @JvmField
        var diffyPower: Double = -.25

        @JvmField
        var linkageRotationMultiplier: Double = 0.5

        @JvmField
        var useTeleopHold: Boolean = true

        @JvmField
        var teleopHoldDuration: Long = 500

        @JvmField
        var minJoystickValue: Double = 0.1

        // delays

        @JvmField
        var transferClawDelay: Long = 250

        @JvmField
        var outtakeFlipDelay: Long = 250

        @JvmField
        var specimenCloseDuration: Long = 250

        @JvmField
        var outtakeDropArmDelay: Long = 250

        @JvmField
        var intakePickupArmDelay: Long = 250

        @JvmField
        var intakePickupClawDelay: Long = 250

        @JvmField
        var baseIntakeDuration: Long = 1250

        @JvmField
        var intakeDuration: Long = 1000

        // <editor-fold desc="Heading Assist">
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
        // </editor-fold>
    }
}
