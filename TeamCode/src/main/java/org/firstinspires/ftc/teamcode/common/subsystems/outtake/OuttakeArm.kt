package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.ServoLimiter
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

enum class OuttakeArmPosition {
    BASE,
    SPECIMEN,
    SPECIMEN_SCORING,
    BASKET,
    PICKUP,
    HUMAN,
    PARK,
}

@Config
class OuttakeArm(
    val robot: Robot,
) : Subsystem() {
    var leftServo: ServoImplEx = (robot.hardware["outtakeArmLeft"] as ServoImplEx).apply { init() }
    var rightServo: ServoImplEx = (robot.hardware["outtakeArmRight"] as ServoImplEx).apply { init(false) }
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

    fun basket() = InstantCommand({ state = OuttakeArmPosition.BASKET }, this)

    fun pickup() = InstantCommand({ state = OuttakeArmPosition.PICKUP }, this)

    fun human() = InstantCommand({ state = OuttakeArmPosition.HUMAN }, this)

    fun park() = InstantCommand({ state = OuttakeArmPosition.PARK }, this)

    companion object {
        @JvmField
        var basePosition = 0.72

        @JvmField
        var basketPosition = 0.55

        @JvmField
        var specimenPosition = 0.44

        @JvmField
        var specimenScoringPosition = 0.77

        @JvmField
        var pickupPosition = 0.89

        @JvmField
        var humanPosition = 0.4

        @JvmField
        var parkPosition = 0.6

        @JvmField
        var maxSpeed = -1.0
    }
}
