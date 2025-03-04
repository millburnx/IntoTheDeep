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
    ALT_SPECIMEN,
    BASKET,
    PICKUP,
    HUMAN,
    PARK,
    HALFBASKET,
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
                OuttakeWristPosition.ALT_SPECIMEN -> altSpecimenPosition
                OuttakeWristPosition.BASKET -> basketPosition
                OuttakeWristPosition.PICKUP -> pickupPosition
                OuttakeWristPosition.HUMAN -> humanPosition
                OuttakeWristPosition.PARK -> parkPosition
                OuttakeWristPosition.HALFBASKET -> halfBasket
            }
        servo.position = target
    }

    fun base() = InstantCommand({ state = OuttakeWristPosition.BASE }, this)

    fun specimen() = InstantCommand({ state = OuttakeWristPosition.SPECIMEN }, this)

    fun altSpecimen() = InstantCommand({ state = OuttakeWristPosition.ALT_SPECIMEN }, this)

    fun basket() = InstantCommand({ state = OuttakeWristPosition.BASKET }, this)

    fun halfBasket() = InstantCommand({ state = OuttakeWristPosition.HALFBASKET }, this)

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
        var altSpecimenPosition = 0.45

        @JvmField
        var pickupPosition = 0.45

        @JvmField
        var humanPosition = 0.7

        @JvmField
        var parkPosition = 0.45

        @JvmField
        var halfBasket = (basketPosition + basePosition) / 2
    }
}
