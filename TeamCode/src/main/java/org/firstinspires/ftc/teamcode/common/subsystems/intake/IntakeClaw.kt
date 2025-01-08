package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init

@Config
class IntakeClaw(val robot: Robot) : Subsystem() {
    val clawServo: ServoImplEx = (robot.hardware["intakeClaw"] as ServoImplEx).apply { init() }

    var isOpen = true

    fun open() {
        isOpen = true
    }

    fun close() {
        isOpen = false
    }

    override fun periodic() {
        clawServo.position = if (isOpen) open else closed
    }

    companion object {
        @JvmField
        var open: Double = 0.3375

        @JvmField
        var closed: Double = 0.625
    }
}