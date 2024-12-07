package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

enum class IntakeArmPosition {
    BASE, EXTENDED, FLOOR
}

class IntakeArm(val robot: Robot) : Subsystem() {
    var leftServo: Servo = robot.hardware["intakeArmLeft"] as Servo
    var rightServo: Servo = robot.hardware["intakeArmRight"] as Servo
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
        var basePosition = 0.0

        @JvmField
        var extendedPosition = 0.5

        @JvmField
        var floorPosition = 0.7
    }
}