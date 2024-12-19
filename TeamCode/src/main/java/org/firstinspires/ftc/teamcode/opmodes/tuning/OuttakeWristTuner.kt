package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWrist
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class OuttakeWristOnly(opmode: OpMode) : Robot(opmode) {
    val outtakeWrist: OuttakeWrist by lazy { OuttakeWrist(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(outtakeWrist) }
}

@TeleOp(name = "Outtake Wrist Tuner")
@Config
class OuttakeWristTuner : OpMode() {
    override val robot by lazy { OuttakeWristOnly(this) }

    override fun exec() {
        robot.outtakeWrist.state = OuttakeWristPosition.entries.toTypedArray()[state]
    }

    companion object {
        @JvmField
        var state: Int = 0
    }
}