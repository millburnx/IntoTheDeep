package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode

@Config
@TeleOp(name = "Basic Teleop")
class BasicTeleop : OpMode() {
    override fun exec() {
        robot.drive.robotCentric(
            gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
        )

        // Outtake
        // Slides
        robot.outtake.slides.target += (gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()) * slideSpeed
        // Arm
//        if (gamepad1.x) {
//            robot.outtake.arm.state = OuttakeArmPosition.BASE
//        } else if (gamepad1.y) {
//            robot.outtake.arm.state = OuttakeArmPosition.BASKET
//        } else if (gamepad1.b) {
//            robot.outtake.arm.state = OuttakeArmPosition.OUT
//        }

//        // Intake
//        // Slides
        if (gamepad1.right_bumper) {
            robot.intake.linkage.target = 1.0
        } else if (gamepad1.left_bumper) {
            robot.intake.linkage.target = 0.0
        }

        if (gamepad1.dpad_up) {
            robot.outtake.slides.target = Slides.max
            robot.intake.linkage.target = 1.0
        } else if (gamepad1.dpad_down) {
            robot.outtake.slides.target = Slides.min
            robot.intake.linkage.target = 0.0
        }
//        // Arm
//        if (gamepad1.dpad_up) {
//            robot.intake.arm.state = IntakeArmPosition.BASE
//        } else if (gamepad1.dpad_left) {
//            robot.intake.arm.state = IntakeArmPosition.EXTENDED
//        } else if (gamepad1.dpad_down) {
//            robot.intake.arm.state = IntakeArmPosition.FLOOR
//        }
//        // diffy
//        if (gamepad1.y) {
//            // pitch up
//        } else if (gamepad1.a) {
//            // pitch down
//        }
//        if (gamepad1.x) {
//            // roll left
//        } else if (gamepad1.b) {
//            // roll right
//        }
//
        robot.telemetry.addLine("Slides Target: ${robot.outtake.slides.target}")
        robot.telemetry.addLine("Slides Position: ${robot.outtake.slides.leftLift.currentPosition}")
        robot.telemetry.addLine("Linkage Target: ${robot.intake.linkage.target}")
        robot.telemetry.addLine("Linkage Position: ${robot.intake.linkage.leftServo.position}")
    }

    companion object {
        @JvmField
        var slideSpeed: Double = 100.0
    }
}
