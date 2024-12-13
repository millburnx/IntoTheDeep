package org.firstinspires.ftc.teamcode.common.commands.outtake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import kotlin.math.absoluteValue

@Config
class SlidesCommand(
    val slides: Slides,
    val target: Double,
    val tolerance: Double = Companion.tolerance
) : CommandBase() {
    init {
        addRequirements(slides)
    }

    override fun execute() {
        slides.target = target
    }

    override fun isFinished(): Boolean {
        // slides.target instead of just target so we don't have to re-clamp or whatever again
        val diff = slides.position - slides.target
        return diff.absoluteValue < tolerance
    }

    companion object {
        @JvmField
        var tolerance: Double = 100.0
    }
}