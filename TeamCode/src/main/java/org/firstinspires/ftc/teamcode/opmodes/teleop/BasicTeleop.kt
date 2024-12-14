package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode

@Config
@TeleOp(name = "Basic Teleop")
class BasicTeleop : OpMode() {
    val triggers by lazy {
        object {
            val linkageRetractTrigger = EdgeDetector({ gamepad1.left_bumper }) {
                robot.intake.linkage.target = 0.0
            }
            val linkageExtensionTrigger = EdgeDetector({ gamepad1.right_bumper }) {
                robot.intake.linkage.target = 1.0
            }
            val armFloor = EdgeDetector({ gamepad1.dpad_down }) {
                robot.intake.arm.state = IntakeArmPosition.FLOOR
            }
            val armUp = EdgeDetector({ gamepad1.dpad_left }) {
                robot.intake.arm.state = IntakeArmPosition.EXTENDED
            }
            val armBase = EdgeDetector({ gamepad1.dpad_right }) {
                robot.intake.arm.state = IntakeArmPosition.BASE
            }
            val intakeRetract = EdgeDetector({ gamepad1.cross }) {
                robot.intake.linkage.target = 0.0
                robot.intake.arm.state = IntakeArmPosition.BASE
            }
            val intakeExtend = EdgeDetector({ gamepad1.triangle }) {
                robot.intake.linkage.target = 1.0
                robot.intake.arm.state = IntakeArmPosition.EXTENDED
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
        robot.outtake.slides.target += slidePower * slideSpeed

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
    }
}
