package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

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
    var leftServo: Servo = (robot.hardware["intakeArmLeft"] as Servo).apply { init() }
    var rightServo: Servo = (robot.hardware["intakeArmRight"] as Servo).apply { init(false) }
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
        var basePosition = 0.45

        @JvmField
        var extendedPosition = 0.575

        @JvmField
        var floorPosition = 0.73

        @JvmField
        var specimenPosition = 0.23
    }
}
