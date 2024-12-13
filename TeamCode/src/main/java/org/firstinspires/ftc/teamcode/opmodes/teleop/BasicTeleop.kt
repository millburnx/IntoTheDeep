package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode

@Config
@TeleOp(name = "Basic Teleop")
class BasicTeleop : OpMode() {

    val linkageRetractTrigger by lazy {
        EdgeDetector(
            { gamepad1.left_bumper },
            { robot.intake.linkage.target = 0.0 },
        )
    }
    val linkageExtensionTrigger by lazy {
        EdgeDetector(
            { gamepad1.right_bumper },
            { robot.intake.linkage.target = 1.0 },
        )
    }

    val fullRetract by lazy {
        EdgeDetector(
            { gamepad1.dpad_down },
            {
                robot.outtake.slides.target = Slides.min
                robot.intake.linkage.target = 0.0
            },
        )
    }
    val fullExtend by lazy {
        EdgeDetector(
            { gamepad1.dpad_up },
            {
                robot.outtake.slides.target = Slides.max
                robot.intake.linkage.target = 1.0
            },
        )
    }

    override fun initialize() {
        super.initialize()
        linkageRetractTrigger
        linkageExtensionTrigger
        fullRetract
        fullExtend
    }

    override fun exec() {
        robot.drive.robotCentric(
            gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
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
