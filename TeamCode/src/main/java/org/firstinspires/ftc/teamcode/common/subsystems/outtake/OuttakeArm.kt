package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

enum class OuttakeArmPosition {
    BASE, OUT, BASKET
}

@Config
class OuttakeArm(val robot: Robot) : Subsystem() {
    var leftServo: Servo = (robot.hardware["outtakeArmLeft"] as Servo).apply { init() }
    var rightServo: Servo = (robot.hardware["outtakeArmRight"] as Servo).apply { init(false) }
    var state: OuttakeArmPosition = OuttakeArmPosition.BASE

    override fun periodic() {
        val target = when (state) {
            OuttakeArmPosition.BASE -> basePosition
            OuttakeArmPosition.OUT -> extendedPosition
            OuttakeArmPosition.BASKET -> basketPosition
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
        var basketPosition = 0.7
    }
}