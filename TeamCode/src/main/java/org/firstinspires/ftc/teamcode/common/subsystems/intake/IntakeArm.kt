package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedServo

enum class IntakeArmPosition {
    BASE,
    EXTENDED,
    FLOOR,
    SWEEP,
}

@Config
class IntakeArm(
    val robot: Robot,
) : Subsystem() {
    val leftServo = CachedServo(robot.hardware, "intakeArmLeft", isForward = true)
    val rightServo = CachedServo(robot.hardware, "intakeArmRight", isForward = false)

    var state: IntakeArmPosition = IntakeArmPosition.BASE

    override fun init() {
        periodic()
    }

    override fun periodic() {
        val target =
            when (state) {
                IntakeArmPosition.BASE -> basePosition
                IntakeArmPosition.EXTENDED -> extendedPosition
                IntakeArmPosition.FLOOR -> floorPosition
                IntakeArmPosition.SWEEP -> sweepPosition
            }
        leftServo.position = target
        rightServo.position = target
    }

    fun base() = InstantCommand({ state = IntakeArmPosition.BASE })

    fun extended() = InstantCommand({ state = IntakeArmPosition.EXTENDED })

    fun floor() = InstantCommand({ state = IntakeArmPosition.FLOOR })

    fun sweep() = InstantCommand({ state = IntakeArmPosition.SWEEP })

    companion object {
        @JvmField
        var basePosition = 0.53

        @JvmField
        var extendedPosition = 0.6

        @JvmField
        var floorPosition = 0.78

        @JvmField
        var sweepPosition = 0.925
    }
}
