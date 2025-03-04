package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class SlidesOnly(
    opmode: OpMode,
) : Robot(opmode) {
    val slides: Slides by lazy { Slides(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(slides) }
}

@TeleOp(name = "Slides Tuner", group = "Tuning")
@Config
class SlidesTuner : OpMode() {
    override val robot by lazy { SlidesOnly(this) }

    override fun exec() {
        robot.slides.target = target

        robot.telemetry.addData("left pos", robot.slides.leftLift.position)
        robot.telemetry.addData("right pos", robot.slides.rightLift.position)
    }

    companion object {
        @JvmField
        var target: Double = 0.0
    }
}
