package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

enum class IntakeArmPosition {
    BASE, EXTENDED, FLOOR
}

@Config
class IntakeArm(val robot: Robot) : Subsystem() {
    var leftServo: Servo = (robot.hardware["intakeArmLeft"] as Servo).apply { init() }
    var rightServo: Servo = (robot.hardware["intakeArmRight"] as Servo).apply { init(false) }
    var state: IntakeArmPosition = IntakeArmPosition.BASE

    override fun periodic() {
        val target = when (state) {
            IntakeArmPosition.BASE -> basePosition
            IntakeArmPosition.EXTENDED -> extendedPosition
            IntakeArmPosition.FLOOR -> floorPosition
        }
        leftServo.position = target
        rightServo.position = target
    }

    companion object {
        @JvmField
        var basePosition = 0.1

        @JvmField
        var extendedPosition = 0.4

        @JvmField
        var floorPosition = 0.7
    }
}