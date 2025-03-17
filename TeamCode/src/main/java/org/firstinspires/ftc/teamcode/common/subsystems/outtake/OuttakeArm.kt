package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedServo
import org.firstinspires.ftc.teamcode.common.utils.hardware.ServoLimiter

enum class OuttakeArmPosition {
    BASE,
    TRANSFER,
    SPECIMEN,
    AUTON_SPECIMEN,
    BASKET,
    PICKUP,
    AUTON_PICKUP,
    PARK,
}

@Config
class OuttakeArm(
    val robot: Robot,
) : Subsystem() {
    val leftServo = CachedServo(robot.hardware, "outtakeArmLeft", true)
    val rightServo = CachedServo(robot.hardware, "outtakeArmRight", true, isForward = false)

    val servoLimiter = ServoLimiter(maxSpeed, robot.deltaTime::deltaTime, basePosition)
    var state: OuttakeArmPosition = OuttakeArmPosition.BASE

    override fun init() {
        periodic()
    }

    override fun periodic() {
        val target =
            when (state) {
                OuttakeArmPosition.BASE -> basePosition
                OuttakeArmPosition.TRANSFER -> transferPosition
                OuttakeArmPosition.SPECIMEN -> specimenPosition
                OuttakeArmPosition.AUTON_SPECIMEN -> autonSpecimenPosition
                OuttakeArmPosition.BASKET -> basketPosition
                OuttakeArmPosition.PICKUP -> pickupPosition
                OuttakeArmPosition.AUTON_PICKUP -> autonPickupPosition
                OuttakeArmPosition.PARK -> parkPosition
            }
        servoLimiter.maxSpeed = maxSpeed
        servoLimiter.update(target)
        leftServo.position = servoLimiter.current
        rightServo.position = servoLimiter.current
    }

    fun base() = InstantCommand({ state = OuttakeArmPosition.BASE }, this)

    fun transfer() = InstantCommand({ state = OuttakeArmPosition.TRANSFER }, this)

    fun specimen() = InstantCommand({ state = OuttakeArmPosition.SPECIMEN }, this)

    fun autonSpecimen() = InstantCommand({ state = OuttakeArmPosition.AUTON_SPECIMEN }, this)

    fun basket() = InstantCommand({ state = OuttakeArmPosition.BASKET }, this)

    fun pickup() = InstantCommand({ state = OuttakeArmPosition.PICKUP }, this)

    fun autonPickup() = InstantCommand({ state = OuttakeArmPosition.AUTON_PICKUP }, this)

    fun park() = InstantCommand({ state = OuttakeArmPosition.PARK }, this)

    companion object {
        @JvmField
        var basePosition = 0.59

        @JvmField
        var transferPosition = 0.62

        @JvmField
        var basketPosition = 0.48

        @JvmField
        var specimenPosition = 0.74

        @JvmField
        var autonSpecimenPosition = 0.26 // different so we can be right against the wall

        @JvmField
        var pickupPosition = 0.26

        @JvmField
        var autonPickupPosition = 0.74

        @JvmField
        var parkPosition = 0.48

        @JvmField
        var maxSpeed = -1.0
    }
}
