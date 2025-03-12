package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedServo

enum class OuttakeWristPosition {
    BASE,
    TRANSFER,
    SPECIMEN,
    AUTON_SPECIMEN,
    BASKET,
    PICKUP,
    PARK,
}

@Config
class OuttakeWrist(
    val robot: Robot,
) : Subsystem() {
    val servo = CachedServo(robot.hardware, "outtakeWrist")
    var state: OuttakeWristPosition = OuttakeWristPosition.BASE

    override fun init() {
        periodic()
    }

    override fun periodic() {
        val target =
            when (state) {
                OuttakeWristPosition.BASE -> basePosition
                OuttakeWristPosition.TRANSFER -> transferPosition
                OuttakeWristPosition.SPECIMEN -> specimenPosition
                OuttakeWristPosition.AUTON_SPECIMEN -> autonSpecimenPosition
                OuttakeWristPosition.BASKET -> basketPosition
                OuttakeWristPosition.PICKUP -> pickupPosition
                OuttakeWristPosition.PARK -> parkPosition
            }
        servo.position = target
    }

    fun base() = InstantCommand({ state = OuttakeWristPosition.BASE }, this)

    fun transfer() = InstantCommand({ state = OuttakeWristPosition.TRANSFER }, this)

    fun specimen() = InstantCommand({ state = OuttakeWristPosition.SPECIMEN }, this)

    fun autonSpecimen() = InstantCommand({ state = OuttakeWristPosition.AUTON_SPECIMEN }, this)

    fun basket() = InstantCommand({ state = OuttakeWristPosition.BASKET }, this)

    fun pickup() = InstantCommand({ state = OuttakeWristPosition.PICKUP }, this)

    fun park() = InstantCommand({ state = OuttakeWristPosition.PARK }, this)

    companion object {
        @JvmField
        var basePosition = 0.8

        @JvmField
        var transferPosition = 0.8

        @JvmField
        var basketPosition = 0.15

        @JvmField
        var specimenPosition = 0.3

        @JvmField
        var autonSpecimenPosition = 0.55

        @JvmField
        var pickupPosition = 0.4

        @JvmField
        var parkPosition = 0.4
    }
}
