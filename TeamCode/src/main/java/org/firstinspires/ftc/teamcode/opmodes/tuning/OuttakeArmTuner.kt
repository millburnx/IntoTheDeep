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

            fun shift(step: Double) {
                if (state == 0) {
                    OuttakeArm.basePosition = (OuttakeArm.basePosition + step).coerceIn(0.0, 1.0)
                } else if (state == 1) {
                    OuttakeArm.transferPosition = (OuttakeArm.transferPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 2) {
                    OuttakeArm.specimenPosition = (OuttakeArm.specimenPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 3) {
                    OuttakeArm.autonSpecimenPosition =
                        (OuttakeArm.autonSpecimenPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 4) {
                    OuttakeArm.basketPosition = (OuttakeArm.basketPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 5) {
                    OuttakeArm.pickupPosition = (OuttakeArm.pickupPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 6) {
                    OuttakeArm.parkPosition = (OuttakeArm.parkPosition + step).coerceIn(0.0, 1.0)
                }
            }

            val moveUp =
                EdgeDetector(gamepad1::dpad_up) {
                    shift(step)
                }

            val moveDown =
                EdgeDetector(gamepad1::dpad_down) {
                    shift(-step)
                }
        }
    }

    override fun exec() {
        triggers

        robot.apply {
            if (state == 0) {
                outtakeArm.state = OuttakeArmPosition.BASE
                telemetry.addData("state", "base")
            } else if (state == 1) {
                outtakeArm.state = OuttakeArmPosition.TRANSFER
                telemetry.addData("state", "transfer")
            } else if (state == 2) {
                outtakeArm.state = OuttakeArmPosition.SPECIMEN
                telemetry.addData("state", "specimen")
            } else if (state == 3) {
                outtakeArm.state = OuttakeArmPosition.AUTON_SPECIMEN
                telemetry.addData("state", "auton specimen")
            } else if (state == 4) {
                outtakeArm.state = OuttakeArmPosition.BASKET
                telemetry.addData("state", "basket")
            } else if (state == 5) {
                outtakeArm.state = OuttakeArmPosition.PICKUP
                telemetry.addData("state", "pickup")
            } else if (state == 6) {
                outtakeArm.state = OuttakeArmPosition.PARK
                telemetry.addData("state", "park")
            }

            telemetry.addData("position", outtakeArm.servoLimiter.current)
        }
    }

    companion object {
        @JvmField
        var state: Int = 0

        @JvmField
        var step: Double = 0.01
    }
}
