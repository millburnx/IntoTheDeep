package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import kotlin.math.absoluteValue

@Config
@TeleOp(name = "Basic Teleop")
class BasicTeleop : OpMode() {
    val triggers by lazy {
        object {
            val linkagePickup = EdgeDetector(
                gamepad1::right_bumper,
                this@BasicTeleop,
                InstantCommand({
                    robot.intake.linkage.target = 1.0
                    robot.intake.arm.state = IntakeArmPosition.EXTENDED
                    robot.intake.diffy.pitch = Diffy.hoverPitch
                    robot.intake.diffy.roll = Diffy.hoverRoll
                    robot.intake.claw.isOpen = true
                }, robot.intake),
                SequentialCommandGroup(
                    InstantCommand({
                        robot.intake.arm.state = IntakeArmPosition.FLOOR
                        robot.intake.diffy.pitch = Diffy.pickupPitch
                    }, robot.intake),
                    WaitCommand(pickupArmDelay),
                    InstantCommand({
                        robot.intake.claw.isOpen = false
                    }, robot.intake),
                    WaitCommand(pickupClawDelay),
                    InstantCommand({
                        robot.intake.linkage.target = 0.0
                        robot.intake.arm.state = IntakeArmPosition.BASE
                        robot.intake.diffy.pitch = Diffy.transferPitch
                        robot.intake.diffy.roll = Diffy.transferRoll
                    }, robot.intake)
                )
            )

            val diffyRotate = EdgeDetector(
                gamepad1::left_bumper,
            ) {
                // safety check
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

            val transfer = EdgeDetector(
                gamepad1::square,
            ) {
                // safety checks
                if (robot.intake.linkage.target != 0.0) return@EdgeDetector
                schedule(
                    SequentialCommandGroup(
                        InstantCommand({
                            robot.outtake.arm.state = OuttakeArmPosition.BASE
                            robot.outtake.wrist.state = OuttakeWristPosition.BASE
                            robot.outtake.claw.isOpen = true
                        }, robot.outtake),
                        WaitCommand(transferDuration),
                        InstantCommand({
                            robot.outtake.claw.isOpen = false
                        }, robot.outtake),
                        WaitCommand(transferClawDelay),
                        InstantCommand({
                            robot.intake.claw.isOpen = true
                        }, robot.intake.claw),
                        WaitCommand(transferPostDelay),
                        InstantCommand({
                            robot.outtake.arm.state = OuttakeArmPosition.BASKET
                            robot.outtake.wrist.state = OuttakeWristPosition.BASKET
                        }, robot.outtake)
                    )
                )
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
        var pickupArmDelay: Long = 250

        @JvmField
        var pickupClawDelay: Long = 250

        @JvmField
        var transferClawDelay: Long = 250

        @JvmField
        var transferDuration: Long = 750

        @JvmField
        var transferPostDelay: Long = 250
    }
}
