package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.ServoLimiter
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

enum class OuttakeArmPosition {
    BASE,
    SPECIMEN,
    BASKET,
    OUT,
    HUMAN,
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
                OuttakeArmPosition.BASKET -> basketPosition
                OuttakeArmPosition.OUT -> extendedPosition
                OuttakeArmPosition.HUMAN -> humanPosition
            }
        servoLimiter.maxSpeed = maxSpeed
        servoLimiter.update(target)
        leftServo.position = servoLimiter.current
        rightServo.position = servoLimiter.current
    }

    companion object {
        @JvmField
        var basePosition = 0.77

        @JvmField
        var basketPosition = 0.65

        @JvmField
        var specimenPosition = 0.5

        @JvmField
        var extendedPosition = 0.89

        @JvmField
        var humanPosition = 0.65

        @JvmField
        var maxSpeed = -1.0
    }
}
