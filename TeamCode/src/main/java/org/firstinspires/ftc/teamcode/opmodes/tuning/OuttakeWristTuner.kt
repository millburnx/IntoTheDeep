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
        if (state == 0) {
            robot.outtakeWrist.state = OuttakeWristPosition.BASE
        } else if (state == 1) {
            robot.outtakeWrist.state = OuttakeWristPosition.BASKET
        } else if (state == 2) {
            robot.outtakeWrist.state = OuttakeWristPosition.OUT
        }
    }

    companion object {
        @JvmField
        var state: Int = 0
    }
}