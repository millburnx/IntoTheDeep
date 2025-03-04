package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class DiffyOnly(
    opmode: OpMode,
) : Robot(opmode) {
    val diffy: Diffy by lazy { Diffy(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(diffy) }
}

@TeleOp(name = "Diffy Tuner", group = "Tuning")
@Config
class DiffyTuner : OpMode() {
    override val robot by lazy { DiffyOnly(this) }

    override fun exec() {
        robot.diffy.pitch = pitch
        robot.diffy.roll = roll

        robot.telemetry.addData("left", robot.diffy.leftServo.position)
        robot.telemetry.addData("right", robot.diffy.rightServo.position)
    }

    companion object {
        @JvmField
        var pitch: Double = 0.0

        @JvmField
        var roll: Double = 0.0
    }
}
