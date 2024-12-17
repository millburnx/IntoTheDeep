package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import kotlin.math.absoluteValue

@Config
@TeleOp(name = "Basic Teleop")
class BasicTeleop : OpMode() {
    val triggers by lazy {
        object {
            val retractIntake = Pair({
                robot.intake.linkage.target = 0.0
                robot.intake.arm.state = IntakeArmPosition.BASE
            }, listOf(robot.intake.linkage, robot.intake.arm))
            val extendIntake = Pair({
                robot.intake.linkage.target = 1.0
                robot.intake.arm.state = IntakeArmPosition.EXTENDED
            }, listOf<Subsystem>())
            val retractSlides = Pair({
                robot.outtake.slides.target = 0.0
            }, listOf(robot.outtake.slides))
            val specimenSlides = Pair({
                robot.outtake.slides.target = Slides.highRung
            }, listOf(robot.outtake.slides))
            val sampleSlides = Pair({
                robot.outtake.slides.target = Slides.highBasket
            }, listOf(robot.outtake.slides))

            val intakeRetract = EdgeDetector({ gamepad1.dpad_right }) {
                schedule(instCmd(retractIntake))
            }
            val intakeExtend = EdgeDetector({ gamepad1.dpad_left }) {
                schedule(ParallelCommandGroup(instCmd(extendIntake), instCmd(retractSlides)))
            }
            val intakePickup = EdgeDetector({ gamepad1.dpad_down }) {
                schedule(
                    SequentialCommandGroup(
                        InstantCommand({
                            robot.intake.arm.state = IntakeArmPosition.FLOOR
                        }),
                        WaitCommand(500),
                        instCmd(retractIntake)
                    )
                )
            }
            val liftRetract = EdgeDetector({ gamepad1.cross }) {
                schedule(instCmd(retractSlides))
            }
            val liftSpecimen = EdgeDetector({ gamepad1.square }) {
                schedule(ParallelCommandGroup(instCmd(specimenSlides), instCmd(retractIntake)))
            }
            val liftBasket = EdgeDetector({ gamepad1.triangle }) {
                schedule(ParallelCommandGroup(instCmd(sampleSlides), instCmd(retractIntake)))
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
            gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble(), -gamepad1.right_stick_x.toDouble()
        )

        // Outtake
        // Slides
        val slidePower = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
//        robot.outtake.slides.target += slidePower * slideSpeed
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
        robot.telemetry.addData("Loop Hertz", 1.0 / robot.deltaTime.deltaTime)
    }

    companion object {
        @JvmField
        var slideSpeed: Double = 100.0

        @JvmStatic
        var slideThreshold: Double = 0.1
    }
}
