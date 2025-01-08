package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

enum class OuttakeWristPosition {
    BASE, SPECIMEN, BASKET, OUT
}

@Config
class OuttakeWrist(val robot: Robot) : Subsystem() {
    var servo: Servo = (robot.hardware["outtakeWrist"] as Servo).apply { init() }
    var state: OuttakeWristPosition = OuttakeWristPosition.BASE

    override fun periodic() {
        val target = when (state) {
            OuttakeWristPosition.BASE -> basePosition
            OuttakeWristPosition.SPECIMEN -> specimenPosition
            OuttakeWristPosition.BASKET -> basketPosition
            OuttakeWristPosition.OUT -> extendedPosition
        }
        servo.position = target
    }

    companion object {
        @JvmField
        var basePosition = 0.8375

        @JvmField
        var specimenPosition = 0.5

        @JvmField
        var basketPosition = 0.4

        @JvmField
        var extendedPosition = 0.625
    }
}