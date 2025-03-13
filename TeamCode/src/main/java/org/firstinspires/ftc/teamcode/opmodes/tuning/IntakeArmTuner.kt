package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArm
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class IntakeArmOnly(
    opmode: OpMode,
) : Robot(opmode) {
    val intakeArm: IntakeArm by lazy { IntakeArm(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(intakeArm) }
}

@TeleOp(name = "Intake Arm Tuner", group = "Tuning")
@Config
class IntakeArmTuner : OpMode() {
    override val robot by lazy { IntakeArmOnly(this) }

    val triggers by lazy {
        object {
            val cycleLeft =
                EdgeDetector(gamepad1::dpad_left) {
                    state = (state - 1) % 3
                }

            val cycleRight =
                EdgeDetector(gamepad1::dpad_right) {
                    state = (state + 1) % 3
                }

            val moveUp =
                EdgeDetector(gamepad1::dpad_up) {
                    if (state == 0) {
                        IntakeArm.basePosition = (IntakeArm.basePosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 1) {
                        IntakeArm.extendedPosition = (IntakeArm.extendedPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 2) {
                        IntakeArm.floorPosition = (IntakeArm.floorPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 3) {
                        IntakeArm.sweepPosition = (IntakeArm.sweepPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 4) {
                        IntakeArm.safePosition = (IntakeArm.safePosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 5) {
                        IntakeArm.safePosition = (IntakeArm.safe2Position + step).coerceIn(0.0, 1.0)
                    }
                }

            val moveDown =
                EdgeDetector(gamepad1::dpad_down) {
                    if (state == 0) {
                        IntakeArm.basePosition = (IntakeArm.basePosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 1) {
                        IntakeArm.extendedPosition = (IntakeArm.extendedPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 2) {
                        IntakeArm.floorPosition = (IntakeArm.floorPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 3) {
                        IntakeArm.sweepPosition = (IntakeArm.sweepPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 4) {
                        IntakeArm.safePosition = (IntakeArm.safePosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 5) {
                        IntakeArm.safePosition = (IntakeArm.safe2Position - step).coerceIn(0.0, 1.0)
                    }
                }
        }
    }

    override fun exec() {
        triggers

        if (state == 0) {
            robot.intakeArm.state = IntakeArmPosition.BASE
            robot.telemetry.addData("state", "base")
        } else if (state == 1) {
            robot.intakeArm.state = IntakeArmPosition.EXTENDED
            robot.telemetry.addData("state", "extended")
        } else if (state == 2) {
            robot.intakeArm.state = IntakeArmPosition.FLOOR
            robot.telemetry.addData("state", "floor")
        } else if (state == 3) {
            robot.intakeArm.state = IntakeArmPosition.SWEEP
            robot.telemetry.addData("state", "sweep")
        } else if (state == 4) {
            robot.intakeArm.state = IntakeArmPosition.SAFE
            robot.telemetry.addData("state", "safe")
        } else if (state == 5) {
            robot.intakeArm.state = IntakeArmPosition.SAFE2
            robot.telemetry.addData("state", "safe2")
        }

        robot.telemetry.addData("position", robot.intakeArm.leftServo.position)
    }

    companion object {
        @JvmField
        var state: Int = 0

        @JvmField
        var step: Double = 0.01
    }
}
