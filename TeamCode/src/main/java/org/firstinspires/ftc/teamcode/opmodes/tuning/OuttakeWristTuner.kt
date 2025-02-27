package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWrist
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class OuttakeWristOnly(
    opmode: OpMode,
) : Robot(opmode) {
    val outtakeWrist: OuttakeWrist by lazy { OuttakeWrist(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(outtakeWrist) }
}

@TeleOp(name = "Outtake Wrist Tuner", group = "Tuning")
@Config
class OuttakeWristTuner : OpMode() {
    override val robot by lazy { OuttakeWristOnly(this) }

    val triggers by lazy {
        object {
            val cycleLeft =
                EdgeDetector(gamepad1::dpad_left) {
                    state = (state - 1) % 7
                }

            val cycleRight =
                EdgeDetector(gamepad1::dpad_right) {
                    state = (state + 1) % 7
                }

            val moveUp =
                EdgeDetector(gamepad1::dpad_up) {
                    if (state == 0) {
                        OuttakeWrist.basePosition = (OuttakeWrist.basePosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 1) {
                        OuttakeWrist.specimenPosition = (OuttakeWrist.specimenPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 2) {
                        OuttakeWrist.specimenPosition = (OuttakeWrist.specimenPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 3) {
                        OuttakeWrist.basketPosition = (OuttakeWrist.basketPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 4) {
                        OuttakeWrist.pickupPosition = (OuttakeWrist.pickupPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 5) {
                        OuttakeWrist.humanPosition = (OuttakeWrist.humanPosition + step).coerceIn(0.0, 1.0)
                    } else if (state == 6) {
                        OuttakeWrist.parkPosition = (OuttakeWrist.parkPosition + step).coerceIn(0.0, 1.0)
                    }
                }

            val moveDown =
                EdgeDetector(gamepad1::dpad_down) {
                    if (state == 0) {
                        OuttakeWrist.basePosition = (OuttakeWrist.basePosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 1) {
                        OuttakeWrist.specimenPosition = (OuttakeWrist.specimenPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 2) {
                        OuttakeWrist.specimenPosition = (OuttakeWrist.specimenPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 3) {
                        OuttakeWrist.basketPosition = (OuttakeWrist.basketPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 4) {
                        OuttakeWrist.pickupPosition = (OuttakeWrist.pickupPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 5) {
                        OuttakeWrist.humanPosition = (OuttakeWrist.humanPosition - step).coerceIn(0.0, 1.0)
                    } else if (state == 6) {
                        OuttakeWrist.parkPosition = (OuttakeWrist.parkPosition - step).coerceIn(0.0, 1.0)
                    }
                }
        }
    }

    override fun exec() {
        triggers

        if (state == 0) {
            robot.outtakeWrist.state = OuttakeWristPosition.BASE
            robot.telemetry.addData("state", "base")
        } else if (state == 1) {
            robot.outtakeWrist.state = OuttakeWristPosition.SPECIMEN
            robot.telemetry.addData("state", "specimen")
        } else if (state == 2) {
            robot.outtakeWrist.state = OuttakeWristPosition.SPECIMEN
            robot.telemetry.addData("state", "specimen scoring")
        } else if (state == 3) {
            robot.outtakeWrist.state = OuttakeWristPosition.BASKET
            robot.telemetry.addData("state", "basket")
        } else if (state == 4) {
            robot.outtakeWrist.state = OuttakeWristPosition.PICKUP
            robot.telemetry.addData("state", "pickup")
        } else if (state == 5) {
            robot.outtakeWrist.state = OuttakeWristPosition.HUMAN
            robot.telemetry.addData("state", "human")
        } else if (state == 6) {
            robot.outtakeWrist.state = OuttakeWristPosition.PARK
            robot.telemetry.addData("state", "park")
        }
        robot.telemetry.addData("position", robot.outtakeWrist.servo.position)
    }

    companion object {
        @JvmField
        var state: Int = 0

        @JvmField
        var step: Double = 0.01
    }
}
