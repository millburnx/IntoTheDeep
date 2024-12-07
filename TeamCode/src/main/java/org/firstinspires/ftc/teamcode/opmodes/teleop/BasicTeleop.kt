package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.common.Robot

class BasicTeleop : CommandOpMode() {
    val robot = Robot(this)
    val telem = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun initialize() {
        robot.init()
    }

    override fun run() {
        super.run()
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

        telem.addLine("Slides Target: ${robot.outtake.slides.target}")
        telem.addLine("Slides Position: ${robot.outtake.slides.leftLift.currentPosition}")
        telem.addLine("Linkage Target: ${robot.intake.linkage.target}")
        telem.addLine("Linkage Position: ${robot.intake.linkage.leftServo.position}")
        telem.update()
    }
}