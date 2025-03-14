package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.conditionalCommand
import org.firstinspires.ftc.teamcode.common.utils.hardware.CachedMotor
import kotlin.math.abs

@Config
class Slides(
    val robot: Robot,
) : Subsystem() {
    enum class State {
        BASE,
        WALL,
        LOW_BASKET,
        HIGH_BASKET,
        HIGH_RUNG,
        HIGH_RUNG_SCORE,
        AUTON_HIGH_RUNG,
        MANUAL,
        DIRECT,
        REZERO,
    }

    var state = State.BASE
        set(value) {
            if (field == value) return
            field = value
        }

    var actualTarget = 0.0

    val left = CachedMotor(robot.hardware, "leftLift", isBrake = true)
    val right = CachedMotor(robot.hardware, "rightLift", isBrake = true)
    val pid = PIDController(kP, kI, kD)
    val position
        get() = left.position + encoderOffset
    var power = 0.0
        set(value) {
            if (field == value) return
            left.power = value
            right.power = value
            field = value
        }

    var target = 0.0
        set(value) {
            field = value.coerceIn(min, max)
        }

    var encoderOffset: Double = 0.0

    fun goTo(target: Double) = SlidesCommand(this, State.DIRECT, target)

    fun goTo(state: State) = SlidesCommand(this, state, target)

    fun direct(power: Double) = SlidesCommand(this, State.DIRECT, power)

    fun reset() =
        SequentialCommandGroup(
            goTo(State.BASE),
            goTo(State.REZERO),
            WaitCommand(rezeroDuration),
            conditionalCommand(
                rezeroCmd(),
            ) { state == State.REZERO },
            goTo(State.BASE),
        )

    fun rezeroCmd() = InstantCommand(this::rezero)

    fun rezero() {
        encoderOffset -= position
    }

    override fun periodic() {
        if (state == State.DIRECT) return
        if (state == State.REZERO) {
            power = rezeroPower
            return
        }

        val target =
            when (state) {
                State.BASE -> min
                State.WALL -> wall
                State.LOW_BASKET -> lowBasket
                State.HIGH_BASKET -> highBasket
                State.HIGH_RUNG -> highRung
                State.HIGH_RUNG_SCORE -> highRungScore
                State.AUTON_HIGH_RUNG -> autonHighRung
                State.MANUAL -> this.target
                else -> return
            }

//        println("slide target $target")

        this.target = target
        this.actualTarget = target

        pid.setPID(kP, kI, kD)
        val power = pid.calculate(position, target) + kF
        val error = target - position

        if (abs(min - target) < 5.0 && abs(error) < 10.0) {
            this.power = 0.0
        } else {
            this.power = power
        }
    }

    companion object {
        @JvmField
        var kP: Double = 0.005

        @JvmField
        var kI: Double = 0.15

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kF: Double = 0.01

        @JvmField
        var min: Double = 1.0

        @JvmField
        var max: Double = 2200.0

        @JvmField
        var highRung: Double = 1300.0

        @JvmField
        var autonHighRung: Double = 1800.0

        @JvmField
        var highRungScore: Double = 2000.0

        @JvmField
        var wall: Double = 170.0

        @JvmField
        var lowBasket: Double = 900.0

        @JvmField
        var highBasket: Double = 2150.0

        @JvmField
        var rezeroPower: Double = -0.5

        @JvmField
        var rezeroDuration: Long = 250
    }
}
