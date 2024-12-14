package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArm
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class IntakeArmOnly(opmode: OpMode) : Robot(opmode) {
    val intakeArm: IntakeArm by lazy { IntakeArm(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(intakeArm) }
}

@TeleOp(name = "Intake Arm Tuner")
@Config
class IntakeArmTuner : OpMode() {
    override val robot by lazy { IntakeArmOnly(this) }

    override fun exec() {
        if (state == 0) {
            robot.intakeArm.state = IntakeArmPosition.BASE
        } else if (state == 1) {
            robot.intakeArm.state = IntakeArmPosition.EXTENDED
        } else if (state == 2) {
            robot.intakeArm.state = IntakeArmPosition.FLOOR
        }
    }

    companion object {
        @JvmField
        var state: Int = 0
    }
}