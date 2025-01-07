package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

enum class OuttakeArmPosition {
    BASE, OUT, BASKET
}

@Config
class OuttakeArm(val robot: Robot) : Subsystem() {
    var leftServo: ServoImplEx = (robot.hardware["outtakeArmLeft"] as ServoImplEx).apply { init() }
    var rightServo: ServoImplEx = (robot.hardware["outtakeArmRight"] as ServoImplEx).apply { init(false) }
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
        var basePosition = 0.59

        @JvmField
        var extendedPosition = 1.0

        @JvmField
        var basketPosition = 0.8
    }
}