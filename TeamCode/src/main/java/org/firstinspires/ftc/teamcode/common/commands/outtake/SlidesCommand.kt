package org.firstinspires.ftc.teamcode.common.commands.outtake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import kotlin.math.absoluteValue

@Config
class SlidesCommand(
    val slides: Slides,
    val state: Slides.State,
    val target: Double = slides.position,
    val tolerance: Double = Companion.tolerance,
) : CommandBase() {
    init {
        addRequirements(slides)
    }

    override fun execute() {
        slides.state = state
        if (state == Slides.State.DIRECT) {
            slides.power = target
        } else {
            slides.target = target
        }
    }

    override fun isFinished(): Boolean {
        // just use a parallel group w/ a wait command, this only has to really run once
        if (state == Slides.State.DIRECT || state == Slides.State.REZERO) return true
        // slides.target instead of just target so we don't have to re-clamp or whatever again
        if (slides.state != state) return true // cancelled/overridden
        val diff = slides.position - slides.actualTarget
        return diff.absoluteValue < tolerance
    }

    companion object {
        @JvmField
        var tolerance: Double = 100.0
    }
}
