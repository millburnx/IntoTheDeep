package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Linkage
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class LinkageOnly(opmode: OpMode) : Robot(opmode) {
    val linkage: Linkage by lazy { Linkage(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(linkage) }
}

@TeleOp(name = "Linkage Tuner")
@Config
class LinkageTuner : OpMode() {
    override val robot by lazy { LinkageOnly(this) }

    override fun exec() {
        robot.linkage.target = target
    }

    companion object {
        @JvmField
        var target: Double = 0.0
    }
}