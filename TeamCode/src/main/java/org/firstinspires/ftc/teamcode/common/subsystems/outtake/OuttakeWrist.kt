package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

enum class OuttakeWristPosition {
    BASE,
    SPECIMEN,
    BASKET,
    OUT,
    HUMAN,
}

@Config
class OuttakeWrist(
    val robot: Robot,
) : Subsystem() {
    var servo: Servo = (robot.hardware["outtakeWrist"] as Servo).apply { init() }
    var state: OuttakeWristPosition = OuttakeWristPosition.BASE

    override fun init() {
        periodic()
    }

    override fun periodic() {
        val target =
            when (state) {
                OuttakeWristPosition.BASE -> basePosition
                OuttakeWristPosition.SPECIMEN -> specimenPosition
                OuttakeWristPosition.BASKET -> basketPosition
                OuttakeWristPosition.OUT -> extendedPosition
                OuttakeWristPosition.HUMAN -> humanPosition
            }
        servo.position = target
    }

    companion object {
        @JvmField
        var basePosition = 0.15

        @JvmField
        var basketPosition = 0.6

        @JvmField
        var specimenPosition = basketPosition

        @JvmField
        var extendedPosition = 0.5

        @JvmField
        var humanPosition = 0.5
    }
}
