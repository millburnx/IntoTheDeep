package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedServo

@Config
class OuttakeClaw(
    val robot: Robot,
) : Subsystem() {
    enum class State {
        OPEN,
        TIGHT_OPEN,
        CLOSED,
    }

    val clawServo = CachedServo(robot.hardware, "outtakeClaw", true)

    var state = State.OPEN

    override fun init() {
        periodic()
    }

    fun open() {
        state = State.OPEN
    }

    fun tightOpen() {
        state = State.TIGHT_OPEN
    }

    fun close() {
        state = State.CLOSED
    }

    override fun periodic() {
        clawServo.position =
            when (state) {
                State.OPEN -> open
                State.TIGHT_OPEN -> tightOpen
                State.CLOSED -> closed
            }
    }

    companion object {
        @JvmField
        var open: Double = 0.95

        @JvmField
        var tightOpen: Double = 0.85

        @JvmField
        var closed: Double = 0.7
    }
}
