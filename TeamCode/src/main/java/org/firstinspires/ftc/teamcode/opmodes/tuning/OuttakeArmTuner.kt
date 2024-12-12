package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArm
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class OuttakeArmOnly(opmode: OpMode) : Robot(opmode) {
    val intakeArm: OuttakeArm by lazy { OuttakeArm(this) }
    override val subsystems: List<Subsystem> = listOf(intakeArm)
}

@TeleOp(name = "Outtake Arm Tuner")
@Config
class OuttakeArmTuner : OpMode() {
    override val robot = OuttakeArmOnly(this)

    override fun exec() {
        robot.intakeArm.state = OuttakeArmPosition.entries.toTypedArray()[state]
    }

    companion object {
        @JvmField
        var state: Int = 0
    }
}