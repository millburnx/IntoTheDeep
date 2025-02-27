package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Linkage
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class LinkageOnly(
    opmode: OpMode,
) : Robot(opmode) {
    val linkage: Linkage by lazy { Linkage(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(linkage) }
}

@TeleOp(name = "Linkage Tuner", group = "Tuning")
@Config
class LinkageTuner : OpMode() {
    override val robot by lazy { LinkageOnly(this) }

    val triggers by lazy {
        object {
            val extend =
                EdgeDetector(gamepad1::right_bumper) {
                    target = 1.0
                }

            val retract =
                EdgeDetector(gamepad1::left_bumper) {
                    target = 0.0
                }

            val moveBack =
                EdgeDetector(gamepad1::dpad_down) {
                    if (target == 0.0) {
                        Linkage.base = (Linkage.base - step).coerceIn(0.0, 1.0)
                    } else {
                        Linkage.full = (Linkage.full - step).coerceIn(0.0, 1.0)
                    }
                }

            val moveForward =
                EdgeDetector(gamepad1::dpad_up) {
                    if (target == 0.0) {
                        Linkage.base = (Linkage.base + step).coerceIn(0.0, 1.0)
                    } else {
                        Linkage.full = (Linkage.full + step).coerceIn(0.0, 1.0)
                    }
                }
        }
    }

    override fun exec() {
        triggers
        robot.linkage.target = target

        robot.telemetry.addData("target", target)
        robot.telemetry.addData("base", Linkage.base)
        robot.telemetry.addData("extended", Linkage.full)
    }

    companion object {
        @JvmField
        var target: Double = 0.0

        @JvmField
        var step: Double = 0.01
    }
}
