package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArm
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class OuttakeArmOnly(
    opmode: OpMode,
) : Robot(opmode) {
    val outtakeArm: OuttakeArm by lazy { OuttakeArm(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(outtakeArm) }
}

@TeleOp(name = "Outtake Arm Tuner")
@Config
class OuttakeArmTuner : OpMode() {
    override val robot by lazy { OuttakeArmOnly(this) }

    override fun exec() {
        if (state == 0) {
            robot.outtakeArm.state = OuttakeArmPosition.BASE
        } else if (state == 1) {
            robot.outtakeArm.state = OuttakeArmPosition.SPECIMEN
        } else if (state == 2) {
            robot.outtakeArm.state = OuttakeArmPosition.BASKET
        } else if (state == 3) {
            robot.outtakeArm.state = OuttakeArmPosition.PICKUP
        }
    }

    companion object {
        @JvmField
        var state: Int = 0
    }
}
