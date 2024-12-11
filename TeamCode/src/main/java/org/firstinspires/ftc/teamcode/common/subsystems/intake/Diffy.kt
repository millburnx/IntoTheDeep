package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

class Diffy(val robot: Robot) : Subsystem() {
    val leftServo: Servo = (robot.hardware["diffyLeft"] as Servo).apply { init() }
    val rightServo: Servo = (robot.hardware["diffyRight"] as Servo).apply { init(false) }

    // each only has an effect range of +/- 0.25 aka 0-0.5 or half
    // as you need to ensure that pitch + roll never exceeds the bounds of 0 to 1
    var pitch: Double = 0.0 // up and down, -1 to 1
    var roll: Double = 0.0 // rotate, -1 to 1

    override fun periodic() {
        val pitchPosition = 0.5 + pitch / 4 // convert to 0.25 to 0.75
        val leftPosition = pitchPosition - roll / 4 // 0 to 1
        val rightPosition = pitchPosition + roll / 4 // 0 to 1
        leftServo.position = leftPosition
        rightServo.position = rightPosition
    }
}