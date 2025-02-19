package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

enum class OuttakeWristPosition {
    BASE,
    SPECIMEN,
    BASKET,
    PICKUP,
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
                OuttakeWristPosition.PICKUP -> pickupPosition
                OuttakeWristPosition.HUMAN -> humanPosition
            }
        servo.position = target
    }

    fun base() = InstantCommand({ state = OuttakeWristPosition.BASE })

    fun specimen() = InstantCommand({ state = OuttakeWristPosition.SPECIMEN })

    fun basket() = InstantCommand({ state = OuttakeWristPosition.BASKET })

    fun pickup() = InstantCommand({ state = OuttakeWristPosition.PICKUP })

    fun human() = InstantCommand({ state = OuttakeWristPosition.HUMAN })

    companion object {
        @JvmField
        var basePosition = 0.15

        @JvmField
        var basketPosition = 0.6

        @JvmField
        var specimenPosition = 0.45

        @JvmField
        var pickupPosition = 0.5

        @JvmField
        var humanPosition = 0.5
    }
}
