package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.AutoPickup
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleCameraRobot
import kotlin.math.absoluteValue

@Config
@TeleOp(name = "New Teleop")
class ControlRewrite : OpMode() {
    override val robot: SampleCameraRobot by lazy { SampleCameraRobot(this) }

    val triggers by lazy {
        object {
            val pickup =
                EdgeDetector(
                    robot.gp1::right_bumper,
                    this@ControlRewrite,
                    SequentialCommandGroup(
                        robot.autoPickup.stop(),
                        ConditionalCommand(
                            SequentialCommandGroup(
                                ParallelCommandGroup(
                                    robot.outtake.arm.base(),
                                    robot.outtake.wrist.base(),
                                ),
                                WaitCommand(outtakeLiftingDuration),
                            ),
                            InstantCommand({}),
                            { robot.intake.arm.state == IntakeArmPosition.SPECIMEN },
                        ),
                        robot.intake.open(),
                        robot.intake.extend(),
                        WaitCommand(AutoPickup.cameraStablizationDuration),
                        robot.autoPickup.startScanning(),
                        robot.autoPickup.rumbleForever,
                    ),
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            robot.autoPickup.cancelRumble(),
                            robot.autoPickup.stopScanning(),
                        ),
                        ConditionalCommand(
                            SequentialCommandGroup(
                                robot.autoPickup.align(),
                                // .withTimeout(AutoPickup.alignmentTimeout)
                                robot.intake.grab(),
                                robot.autoPickup.stop(),
                                // transfer
                                ParallelCommandGroup(
                                    robot.intake.retract(),
                                    robot.outtake.open(),
                                    robot.outtake.base(),
                                ),
                                robot.outtake.close(),
                                WaitCommand(transferClawDelay),
                                robot.intake.open(),
                                WaitCommand(outtakeFlipDelay),
                                ParallelCommandGroup(
                                    robot.outtake.arm.basket(),
                                    robot.outtake.wrist.basket(),
                                ),
                            ),
                            robot.intake.retract(),
                            { robot.autoPickup.lastTarget != null },
                        ),
                    ),
                )

            // samples
            val lowBasket =
                EdgeDetector(
                    robot.gp1::dpad_down,
                    this@ControlRewrite,
                    SequentialCommandGroup(
                        SlidesCommand(robot.outtake.slides, Slides.lowBasket),
                        robot.outtake.arm.basket(),
                        robot.outtake.wrist.basket(),
                    ),
                )

            val highBasket =
                EdgeDetector(
                    robot.gp1::dpad_up,
                    this@ControlRewrite,
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
                    this@ControlRewrite,
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
                    ),
                    SequentialCommandGroup(
                        robot.outtake.close(),
                        WaitCommand(specimenCloseDuration),
                        SlidesCommand(robot.outtake.slides, Slides.wall),
                        ParallelCommandGroup(
                            robot.outtake.arm.specimen(),
                            robot.outtake.wrist.specimen(),
                        ),
                    ),
                )

            val hpDrop =
                EdgeDetector(
                    robot.gp1::square,
                    this@ControlRewrite,
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            robot.outtake.base(),
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
                    this@ControlRewrite,
                    ConditionalCommand(
                        SequentialCommandGroup(
                            robot.outtake.open(),
                        ),
                        SequentialCommandGroup(
                            robot.outtake.arm.specimenScoring(),
                        ),
                        { robot.outtake.arm.state == OuttakeArmPosition.BASKET },
                    ),
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
                                    { robot.intake.arm.state != IntakeArmPosition.SPECIMEN },
                                ),
                            ),
                            ParallelCommandGroup(
                                robot.outtake.arm.pickup(),
                                robot.outtake.wrist.pickup(),
                            ),
                            SlidesCommand(robot.outtake.slides, Slides.min),
                            robot.outtake.slides.reset(),
                        ),
                        { robot.outtake.arm.state == OuttakeArmPosition.BASKET },
                    ),
                )
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

        if (!robot.pidManager.isOn) {
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

        robot.telemetry.addData("pid", robot.pidManager.isOn)
        robot.telemetry.addData("pid target", robot.pidManager.target)
        robot.telemetry.addData("Delta Time", robot.deltaTime.deltaTime)
        robot.telemetry.addData("Loop Hertz", 1.0 / robot.deltaTime.deltaTime)
        robot.telemetry.addData("slides", robot.outtake.slides.position)
        robot.telemetry.addData("slides offset", robot.outtake.slides.encoderOffset)
    }

    companion object {
        @JvmField
        var slideThreshold: Double = 0.1

        @JvmField
        var fieldCentric: Boolean = false

        // delays

        @JvmField
        var transferClawDelay: Long = 250

        @JvmField
        var outtakeFlipDelay: Long = 250

        @JvmField
        var intakeLoweringDuration: Long = 750

        @JvmField
        var outtakeLiftingDuration: Long = 500

        @JvmField
        var specimenCloseDuration: Long = 500
    }
}
