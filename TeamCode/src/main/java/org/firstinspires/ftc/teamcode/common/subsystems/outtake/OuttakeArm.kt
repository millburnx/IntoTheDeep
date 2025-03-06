package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.CachedServo
import org.firstinspires.ftc.teamcode.common.utils.ServoLimiter
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

enum class OuttakeArmPosition {
    BASE,
    SPECIMEN,
    SPECIMEN_SCORING,
    ALT_SPECIMEN,
    BASKET,
    PICKUP,
    HUMAN,
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
                OuttakeArmPosition.SPECIMEN -> specimenPosition
                OuttakeArmPosition.SPECIMEN_SCORING -> specimenScoringPosition
                OuttakeArmPosition.ALT_SPECIMEN -> altSpecimenPosition
                OuttakeArmPosition.BASKET -> basketPosition
                OuttakeArmPosition.PICKUP -> pickupPosition
                OuttakeArmPosition.HUMAN -> humanPosition
                OuttakeArmPosition.PARK -> parkPosition
            }
        servoLimiter.maxSpeed = maxSpeed
        servoLimiter.update(target)
        leftServo.position = servoLimiter.current
        rightServo.position = servoLimiter.current
    }

    fun base() = InstantCommand({ state = OuttakeArmPosition.BASE }, this)

    fun specimen() = InstantCommand({ state = OuttakeArmPosition.SPECIMEN }, this)

    fun specimenScoring() = InstantCommand({ state = OuttakeArmPosition.SPECIMEN_SCORING }, this)

    fun altSpecimen() = InstantCommand({ state = OuttakeArmPosition.ALT_SPECIMEN }, this)

    fun basket() = InstantCommand({ state = OuttakeArmPosition.BASKET }, this)

    fun pickup() = InstantCommand({ state = OuttakeArmPosition.PICKUP }, this)

    fun human() = InstantCommand({ state = OuttakeArmPosition.HUMAN }, this)

    fun park() = InstantCommand({ state = OuttakeArmPosition.PARK }, this)

    companion object {
        @JvmField
        var basePosition = 0.83

        @JvmField
        var basketPosition = 0.5

        @JvmField
        var specimenPosition = 0.74

        @JvmField
        var specimenScoringPosition = specimenPosition

        @JvmField
        var altSpecimenPosition = 0.74

        @JvmField
        var pickupPosition = 0.26

        @JvmField
        var humanPosition = 0.27

        @JvmField
        var parkPosition = 0.48

        @JvmField
        var maxSpeed = -1.0
    }
}
