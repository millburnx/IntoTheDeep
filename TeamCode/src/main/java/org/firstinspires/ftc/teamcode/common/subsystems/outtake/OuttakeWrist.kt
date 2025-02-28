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
    PARK,
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
                OuttakeWristPosition.PARK -> parkPosition
            }
        servo.position = target
    }

    fun base() = InstantCommand({ state = OuttakeWristPosition.BASE }, this)

    fun specimen() = InstantCommand({ state = OuttakeWristPosition.SPECIMEN }, this)

    fun basket() = InstantCommand({ state = OuttakeWristPosition.BASKET }, this)

    fun pickup() = InstantCommand({ state = OuttakeWristPosition.PICKUP }, this)

    fun human() = InstantCommand({ state = OuttakeWristPosition.HUMAN }, this)

    fun park() = InstantCommand({ state = OuttakeWristPosition.PARK }, this)

    companion object {
        @JvmField
        var basePosition = 0.125

        @JvmField
        var basketPosition = 0.6

        @JvmField
        var specimenPosition = 0.45

        @JvmField
        var pickupPosition = 0.45

        @JvmField
        var humanPosition = 0.7

        @JvmField
        var parkPosition = 0.45
    }
}
