package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedServo

enum class IntakeClawState {
    OPEN,
    CLOSED,
    LOOSE,
}

@Config
class IntakeClaw(
    val robot: Robot,
) : Subsystem() {
    val clawServo = CachedServo(robot.hardware, "intakeClaw", true)

    var state = IntakeClawState.OPEN

    fun open() {
        state = IntakeClawState.OPEN
    }

    fun loose() = InstantCommand({ state = IntakeClawState.LOOSE })

    fun close() {
        state = IntakeClawState.CLOSED
    }

    override fun init() {
        periodic()
    }

    override fun periodic() {
//        clawServo.position = if (isOpen) open else closed
        clawServo.position =
            when (state) {
                IntakeClawState.OPEN -> open
                IntakeClawState.CLOSED -> closed
                IntakeClawState.LOOSE -> loose
            }
    }

    companion object {
        @JvmField
        var open: Double = 0.5

        @JvmField
        var closed: Double = 0.1

        @JvmField
        var loose: Double = 0.18
    }
}
