package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArm
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class OuttakeArmOnly(
    opmode: OpMode,
) : Robot(opmode) {
    val outtakeArm: OuttakeArm by lazy { OuttakeArm(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(outtakeArm) }
}

@TeleOp(name = "Outtake Arm Tuner", group = "Tuning")
@Config
class OuttakeArmTuner : OpMode() {
    override val robot by lazy { OuttakeArmOnly(this) }

    val triggers by lazy {
        object {
            val cycleLeft =
                EdgeDetector(gamepad1::dpad_left) {
                    state = (state - 1) % 8
                }

            val cycleRight =
                EdgeDetector(gamepad1::dpad_right) {
                    state = (state + 1) % 8
                }

            val moveUp =
                EdgeDetector(gamepad1::dpad_up) {
                    if (state == 0) {
                        OuttakeArm.basePosition = (OuttakeArm.basePosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 1) {
                        OuttakeArm.specimenPosition = (OuttakeArm.specimenPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 2) {
                        OuttakeArm.specimenScoringPosition =
                            (OuttakeArm.specimenScoringPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 3) {
                        OuttakeArm.altSpecimenPosition = (OuttakeArm.altSpecimenPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 4) {
                        OuttakeArm.basketPosition = (OuttakeArm.basketPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 5) {
                        OuttakeArm.pickupPosition = (OuttakeArm.pickupPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 6) {
                        OuttakeArm.humanPosition = (OuttakeArm.humanPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 7) {
                        OuttakeArm.parkPosition = (OuttakeArm.parkPosition + step).coerceIn(0.0, 1.0)
                    }
                }

            val moveDown =
                EdgeDetector(gamepad1::dpad_down) {
                    if (state == 0) {
                        OuttakeArm.basePosition = (OuttakeArm.basePosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 1) {
                        OuttakeArm.specimenPosition = (OuttakeArm.specimenPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 2) {
                        OuttakeArm.specimenScoringPosition =
                            (OuttakeArm.specimenScoringPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 3) {
                        OuttakeArm.altSpecimenPosition = (OuttakeArm.altSpecimenPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 4) {
                        OuttakeArm.basketPosition = (OuttakeArm.basketPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 5) {
                        OuttakeArm.pickupPosition = (OuttakeArm.pickupPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 6) {
                        OuttakeArm.humanPosition = (OuttakeArm.humanPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 7) {
                        OuttakeArm.parkPosition = (OuttakeArm.parkPosition - step).coerceIn(0.0, 1.0)
                    }
                }
        }
    }

    override fun exec() {
        triggers

        if (state == 0) {
            robot.outtakeArm.state = OuttakeArmPosition.BASE
            robot.telemetry.addData("state", "base")
        } else if (state == 1) {
            robot.outtakeArm.state = OuttakeArmPosition.SPECIMEN
            robot.telemetry.addData("state", "specimen")
        } else if (state == 2) {
            robot.outtakeArm.state = OuttakeArmPosition.SPECIMEN_SCORING
            robot.telemetry.addData("state", "specimen scoring")
        } else if (state == 3) {
            robot.outtakeArm.state = OuttakeArmPosition.ALT_SPECIMEN
            robot.telemetry.addData("state", "alt specimen")
        } else if (state == 4) {
            robot.outtakeArm.state = OuttakeArmPosition.BASKET
            robot.telemetry.addData("state", "basket")
        } else if (state == 5) {
            robot.outtakeArm.state = OuttakeArmPosition.PICKUP
            robot.telemetry.addData("state", "pickup")
        } else if (state == 6) {
            robot.outtakeArm.state = OuttakeArmPosition.HUMAN
            robot.telemetry.addData("state", "human")
        } else if (state == 7) {
            robot.outtakeArm.state = OuttakeArmPosition.PARK
            robot.telemetry.addData("state", "park")
        }
        robot.telemetry.addData("position", robot.outtakeArm.servoLimiter.current)
    }

    companion object {
        @JvmField
        var state: Int = 0

        @JvmField
        var step: Double = 0.01
    }
}
