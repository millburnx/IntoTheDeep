package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.CachedServo
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

@Config
class OuttakeClaw(
    val robot: Robot,
) : Subsystem() {
    val clawServo = CachedServo(robot.hardware, "outtakeClaw", true)

    var isOpen = true

    fun open() {
        isOpen = true
    }

    fun close() {
        isOpen = false
    }

    override fun init() {
        periodic()
    }

    override fun periodic() {
        clawServo.position = if (isOpen) open else closed
    }

    companion object {
        @JvmField
        var open: Double = 0.65

        @JvmField
        var closed: Double = 0.41
    }
}
