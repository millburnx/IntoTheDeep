package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Utils
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

@Config
class Linkage(val robot: Robot) : Subsystem() {
    var target: Double = 0.0 // 0 to 1
    var leftServo: Servo = robot.hardware["linkageLeft"] as Servo
    val rightServo: Servo = robot.hardware["linkageRight"] as Servo

    override fun periodic() {
        val position = Utils.lerp(starting, starting + range, target)
        leftServo.position = position
        rightServo.position = 1.0 - position
    }

    companion object {
        @JvmField
        var starting = 0.165

        @JvmField
        var range = 0.5 - 0.165
    }
}