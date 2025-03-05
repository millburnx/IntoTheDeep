package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.CachedServo
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

enum class IntakeArmPosition {
    BASE,
    EXTENDED,
    FLOOR,
    SPECIMEN,
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
                IntakeArmPosition.SPECIMEN -> specimenPosition
            }
        leftServo.position = target
        rightServo.position = target
    }

    fun base() = InstantCommand({ state = IntakeArmPosition.BASE })

    fun extended() = InstantCommand({ state = IntakeArmPosition.EXTENDED })

    fun floor() = InstantCommand({ state = IntakeArmPosition.FLOOR })

    fun specimen() = InstantCommand({ state = IntakeArmPosition.SPECIMEN })

    companion object {
        @JvmField
        var basePosition = 0.4425

        @JvmField
        var extendedPosition = 0.525

        @JvmField
        var floorPosition = 0.78

        @JvmField
        var specimenPosition = 0.22
    }
}
