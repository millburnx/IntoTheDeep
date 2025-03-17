package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedServo

enum class IntakeArmPosition {
    BASE,
    EXTENDED,
    EXTENDED_MANUAL,
    FLOOR,
    SWEEP,
    SAFE,
    SAFE2,
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
                IntakeArmPosition.EXTENDED_MANUAL -> extendedManualPosition
                IntakeArmPosition.FLOOR -> floorPosition
                IntakeArmPosition.SWEEP -> sweepPosition
                IntakeArmPosition.SAFE -> safePosition
                IntakeArmPosition.SAFE2 -> safe2Position
            }
        leftServo.position = target
        rightServo.position = target
    }

    fun base() = InstantCommand({ state = IntakeArmPosition.BASE })

    fun extended() = InstantCommand({ state = IntakeArmPosition.EXTENDED })

    fun extendedManual() = InstantCommand({ state = IntakeArmPosition.EXTENDED_MANUAL })

    fun floor() = InstantCommand({ state = IntakeArmPosition.FLOOR })

    fun sweep() = InstantCommand({ state = IntakeArmPosition.SWEEP })

    fun safe() = InstantCommand({ state = IntakeArmPosition.SAFE })

    fun safe2() = InstantCommand({ state = IntakeArmPosition.SAFE2 })

    companion object {
        @JvmField
        var basePosition = 0.41

        @JvmField
        var extendedPosition = 0.6

        @JvmField
        var extendedManualPosition = .68

        @JvmField
        var floorPosition = 0.78

        @JvmField
        var sweepPosition = 0.925

        @JvmField
        var safePosition = 0.45

        @JvmField
        var safe2Position = 0.3
    }
}
