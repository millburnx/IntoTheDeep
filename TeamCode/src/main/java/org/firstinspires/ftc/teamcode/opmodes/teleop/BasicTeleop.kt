package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArm
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.roundToLong

@Config
@TeleOp(name = "Basic Teleop")
class BasicTeleop : OpMode() {
    val triggers by lazy {
        object {
            val linkagePickup = EdgeDetector(
                gamepad1::right_bumper,
                this@BasicTeleop,
                ParallelCommandGroup(
                    InstantCommand({
                        robot.intake.linkage.target = 1.0
                        robot.intake.arm.state = IntakeArmPosition.EXTENDED
                        robot.intake.diffy.pitch = Diffy.hoverPitch
                        robot.intake.diffy.roll = Diffy.hoverRoll
                        robot.intake.claw.isOpen = true
                    }, robot.intake),
                    SlidesCommand(robot.outtake.slides, Slides.min)
                ),
                SequentialCommandGroup(
                    InstantCommand({
                        robot.intake.arm.state = IntakeArmPosition.FLOOR
                        robot.intake.diffy.pitch = Diffy.pickupPitch
                    }, robot.intake),
                    WaitCommand(intakePickupArmDelay),
                    InstantCommand({
                        robot.intake.claw.isOpen = false
                    }, robot.intake),
                    WaitCommand(intakePickupClawDelay),
                    InstantCommand({
                        robot.intake.linkage.target = 0.0
                        robot.intake.arm.state = IntakeArmPosition.BASE
                        robot.intake.diffy.pitch = Diffy.transferPitch
                        robot.intake.diffy.roll = Diffy.transferRoll
                    }, robot.intake),
                    WaitCommand(intakeDuration),
                    InstantCommand({
                        robot.outtake.claw.isOpen = false
                    }, robot.outtake),
                    WaitCommand(transferClawDelay),
                    InstantCommand({
                        robot.intake.claw.isOpen = true
                    }, robot.intake.claw),
                    InstantCommand({
                        robot.outtake.arm.state = OuttakeArmPosition.BASE
                        robot.outtake.wrist.state = OuttakeWristPosition.BASE
                        robot.outtake.claw.isOpen = true
                    }, robot.outtake),
                )
            )

            val diffyRotate = EdgeDetector(
                gamepad1::left_bumper,
            ) {
                // prevent rotating while not picking up
                if (robot.intake.arm.state == IntakeArmPosition.BASE) return@EdgeDetector
                schedule(
                    InstantCommand({
                        if (robot.intake.diffy.roll == Diffy.hoverRoll) {
                            robot.intake.diffy.roll = Diffy.roll45
                        } else if (robot.intake.diffy.roll == Diffy.roll45) {
                            robot.intake.diffy.roll = Diffy.roll90
                        } else {
                            robot.intake.diffy.roll = Diffy.hoverRoll
                        }
                    }, robot.intake.diffy)
                )
            }

            val specimenPickup = EdgeDetector(
                gamepad1::circle,
                {
                    val atBase = robot.outtake.arm.state == OuttakeArmPosition.BASE
                    val baseToBasket = if (atBase) {
                        SequentialCommandGroup(
                            InstantCommand({
                                robot.outtake.arm.state = OuttakeArmPosition.BASKET
                                robot.outtake.wrist.state = OuttakeWristPosition.BASKET
                            }, robot.outtake),
                            WaitCommand(
                                estimateDuration(
                                    OuttakeArm.basePosition,
                                    OuttakeArm.basketPosition,
                                    OuttakeArm.maxSpeed
                                )
                            )
                        )
                    } else {
                        InstantCommand({})
                    }
                    schedule(
                        SequentialCommandGroup(
                            InstantCommand({ robot.outtake.claw.isOpen = true }, robot.outtake),
                            baseToBasket,
                            InstantCommand({
                                robot.outtake.arm.state = OuttakeArmPosition.OUT
                                robot.outtake.wrist.state = OuttakeWristPosition.OUT
                            }, robot.outtake),
                        )
                    )
                },
                {
                    schedule(
                        SequentialCommandGroup(
                            InstantCommand({ robot.outtake.claw.isOpen = false }, robot.outtake),
                            WaitCommand(outtakePickupClawDelay),
                            InstantCommand({
                                robot.outtake.arm.state = OuttakeArmPosition.BASKET
                                robot.outtake.wrist.state = OuttakeWristPosition.BASKET
                            }, robot.outtake)
                        )
                    )
                }
            )

            val outtakeClaw = EdgeDetector(
                gamepad1::triangle,
            ) {
                // prevent dropping in bot
                if (robot.outtake.arm.state == OuttakeArmPosition.BASE) return@EdgeDetector
                schedule(
                    InstantCommand({
                        robot.outtake.claw.isOpen = true
                    }, robot.outtake.claw)
                )
            }

            val specimenScore = EdgeDetector(
                gamepad1::dpad_left,
                this@BasicTeleop,
                InstantCommand({
                    robot.outtake.arm.state = OuttakeArmPosition.SPECIMEN
                    robot.outtake.wrist.state = OuttakeWristPosition.SPECIMEN
                }, robot.outtake)
            )
            val base = EdgeDetector(
                gamepad1::dpad_down,
                this@BasicTeleop,
                SlidesCommand(robot.outtake.slides, Slides.min)
            )
            val sampleScore = EdgeDetector(
                gamepad1::dpad_up,
                this@BasicTeleop,
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
                        SlidesCommand(robot.outtake.slides, Slides.highBasket)
                    )
                )
            )

            fun estimateDuration(starting: Double, ending: Double, speed: Double): Long {
                val diff = abs(starting - ending)
                return (diff / speed * 1000).roundToLong()
            }

            fun instCmd(cmd: Pair<() -> Unit, List<Subsystem>>): InstantCommand {
                return InstantCommand(cmd.first, *cmd.second.toTypedArray())
            }
        }
    }

    override fun initialize() {
        super.initialize()
        triggers // lazy loads all the triggers w/o having to make each lazy loaded
    }

    override fun exec() {
        robot.drive.robotCentric(
            gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
        )

        // Slides
        val slidePower = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
        if (slidePower.absoluteValue > slideThreshold) {
            robot.outtake.slides.isManual = true
            robot.outtake.slides.manualPower = slidePower
        } else {
            robot.outtake.slides.isManual = false
        }

        robot.telemetry.addData("Slides Target: ", robot.outtake.slides.target)
        robot.telemetry.addData("Slides Position: ", robot.outtake.slides.leftLift.currentPosition)
        robot.telemetry.addData("Linkage Target: ", robot.intake.linkage.target)
        robot.telemetry.addData("Linkage Position:", robot.intake.linkage.leftServo.position)
        robot.telemetry.addData("Delta Time", robot.deltaTime.deltaTime)
        robot.telemetry.addData("Intake Claw Open:", robot.intake.claw.isOpen)
        robot.telemetry.addData("Outtake Claw Open:", robot.outtake.claw.isOpen)
        robot.telemetry.addData("Loop Hertz", 1.0 / robot.deltaTime.deltaTime)
    }

    companion object {
        @JvmField
        var slideThreshold: Double = 0.1

        @JvmField
        var intakePickupArmDelay: Long = 250

        @JvmField
        var intakePickupClawDelay: Long = 250

        @JvmField
        var transferClawDelay: Long = 250

        @JvmField
        var transferDuration: Long = 1500

        @JvmField
        var transferPostDelay: Long = 250

        @JvmField
        var outtakePickupClawDelay: Long = 250

        @JvmField
        var intakeDuration: Long = 1500
    }
}
