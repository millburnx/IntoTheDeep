package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArm
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class IntakeArmOnly(opmode: OpMode) : Robot(opmode) {
    val intakeArm: IntakeArm = IntakeArm(this)
    override val subsystems: List<Subsystem> = listOf(intakeArm)
}

@TeleOp(name = "Intake Arm Tuner")
@Config
class IntakeArmTuner : OpMode() {
    override val robot = IntakeArmOnly(this)

    override fun exec() {
        robot.intakeArm.state = IntakeArmPosition.entries.toTypedArray()[state]
    }

    companion object {
        @JvmField
        var state: Int = 0
    }
}