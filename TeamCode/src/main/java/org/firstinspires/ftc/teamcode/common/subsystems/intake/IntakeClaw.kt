package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedServo

@Config
class IntakeClaw(
    val robot: Robot,
) : Subsystem() {
    enum class State {
        OPEN,
        CLOSED,
        LOOSE,
    }

    val clawServo = CachedServo(robot.hardware, "intakeClaw", true)

    var state = State.OPEN

    fun open() {
        state = State.OPEN
    }

    fun loose() = InstantCommand({ state = State.LOOSE })

    fun close() {
        state = State.CLOSED
    }

    override fun init() {
        periodic()
    }

    override fun periodic() {
//        clawServo.position = if (isOpen) open else closed
        clawServo.position =
            when (state) {
                State.OPEN -> open
                State.CLOSED -> closed
                State.LOOSE -> loose
            }
    }

    companion object {
        @JvmField
        var open: Double = 0.5

        @JvmField
        var closed: Double = 0.05

        @JvmField
        var loose: Double = 0.13
    }
}
