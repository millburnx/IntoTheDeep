package org.firstinspires.ftc.teamcode.opmodes.teleop

import org.firstinspires.ftc.teamcode.common.utils.OpMode

class BasicTeleop : OpMode() {
    override fun exec() {
        robot.drive.robotCentric(
            gamepad1.left_stick_y.toDouble(),
            gamepad1.left_stick_x.toDouble(),
            gamepad1.right_stick_x.toDouble()
        )

        robot.outtake.slides.target += gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
        if (gamepad1.right_bumper) {
            robot.intake.linkage.target = 1.0
        } else if (gamepad1.left_bumper) {
            robot.intake.linkage.target = 0.0
        }

        robot.telemetry.addLine("Slides Target: ${robot.outtake.slides.target}")
        robot.telemetry.addLine("Slides Position: ${robot.outtake.slides.leftLift.currentPosition}")
        robot.telemetry.addLine("Linkage Target: ${robot.intake.linkage.target}")
        robot.telemetry.addLine("Linkage Position: ${robot.intake.linkage.leftServo.position}")
    }
}