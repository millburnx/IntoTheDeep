package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.millburnx.utils.Utils
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.CachedServo
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleopBlue.Companion.baseIntakeDuration
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleopBlue.Companion.intakeDuration

@Config
class Linkage(
    val robot: Robot,
) : Subsystem() {
    var target: Double = 0.0 // 0 to 1
    val leftServo = CachedServo(robot.hardware, "linkageLeft")
    val rightServo = CachedServo(robot.hardware, "linkageRight", isForward = false)

    override fun init() {
        periodic()
    }

    fun extendAsync() = InstantCommand({ target = 1.0 }, this)

    fun extend() =
        ParallelCommandGroup(
            WaitCommand(
                (intakeDuration * (1 - robot.intake.linkage.target)).toLong().coerceAtLeast(baseIntakeDuration),
            ),
            extendAsync(),
        )

    fun retractAsync() = InstantCommand({ target = 0.0 }, this)

    fun retract() =
        ParallelCommandGroup(
            WaitCommand((intakeDuration * robot.intake.linkage.target).toLong().coerceAtLeast(baseIntakeDuration)),
            retractAsync(),
        )

    override fun periodic() {
        if (!enabled) return
        val position = Utils.lerp(base, full, target)
        leftServo.position = position
        rightServo.position = position
    }

    companion object {
        @JvmField
        var base = 0.3

        @JvmField
        var full = 0.65

        @JvmField
        var enabled: Boolean = true
    }
}
