package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Utils
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

@Config
class Linkage(val robot: Robot) : Subsystem() {
    var target: Double = 0.0 // 0 to 1
    var leftServo: Servo = (robot.hardware["linkageLeft"] as Servo).apply { init() }
    val rightServo: Servo = (robot.hardware["linkageRight"] as Servo).apply { init(false) }

    override fun periodic() {
        if (!enabled) return
        val position = Utils.lerp(base, full, target)
        leftServo.position = position
        rightServo.position = position
    }

    companion object {
        @JvmField
        var base = 0.35

        @JvmField
        var full = 0.65

        @JvmField
        var enabled: Boolean = true
    }
}