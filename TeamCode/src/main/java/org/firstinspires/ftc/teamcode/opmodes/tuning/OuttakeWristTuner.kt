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
                    state = (state - 1) % 8
                }

            val cycleRight =
                EdgeDetector(gamepad1::dpad_right) {
                    state = (state + 1) % 8
                }

            fun shift(step: Double) {
                if (state == 0) {
                    OuttakeWrist.basePosition = (OuttakeWrist.basePosition + step).coerceIn(0.0, 1.0)
                } else if (state == 1) {
                    OuttakeWrist.transferPosition = (OuttakeWrist.transferPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 2) {
                    OuttakeWrist.specimenPosition = (OuttakeWrist.specimenPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 3) {
                    OuttakeWrist.autonSpecimenPosition =
                        (OuttakeWrist.autonSpecimenPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 4) {
                    OuttakeWrist.basketPosition = (OuttakeWrist.basketPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 5) {
                    OuttakeWrist.pickupPosition = (OuttakeWrist.pickupPosition + step).coerceIn(0.0, 1.0)
                } else if (state == 6) {
                    OuttakeWrist.parkPosition = (OuttakeWrist.parkPosition + step).coerceIn(0.0, 1.0)
                }
            }

            val moveUp = EdgeDetector(gamepad1::dpad_up) { shift(step) }

            val moveDown = EdgeDetector(gamepad1::dpad_down) { shift(-step) }
        }
    }

    override fun exec() {
        triggers

        robot.apply {
            if (state == 0) {
                outtakeWrist.state = OuttakeWristPosition.BASE
                telemetry.addData("state", "base")
            } else if (state == 1) {
                outtakeWrist.state = OuttakeWristPosition.TRANSFER
                telemetry.addData("state", "transfer")
            } else if (state == 2) {
                outtakeWrist.state = OuttakeWristPosition.SPECIMEN
                telemetry.addData("state", "specimen")
            } else if (state == 3) {
                outtakeWrist.state = OuttakeWristPosition.AUTON_SPECIMEN
                telemetry.addData("state", "auton specimen")
            } else if (state == 4) {
                outtakeWrist.state = OuttakeWristPosition.BASKET
                telemetry.addData("state", "basket")
            } else if (state == 5) {
                outtakeWrist.state = OuttakeWristPosition.PICKUP
                telemetry.addData("state", "pickup")
            } else if (state == 6) {
                outtakeWrist.state = OuttakeWristPosition.PARK
                telemetry.addData("state", "park")
            }

            telemetry.addData("position", outtakeWrist.servo.position)
        }
    }

    companion object {
        @JvmField
        var state: Int = 0

        @JvmField
        var step: Double = 0.01
    }
}
