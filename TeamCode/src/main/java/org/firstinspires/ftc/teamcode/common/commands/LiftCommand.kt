package org.firstinspires.ftc.teamcode.common.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import kotlin.math.abs

class LiftCommand(val lift: Lift, val target: Int, val threshold: Int = 50) : CommandBase() {
    init {
        addRequirements(lift)
    }

    override fun initialize() {
    }


    override fun execute() {
        lift.target = target
        // no need to call .run() since lift will already manage running
    }

    override fun isFinished(): Boolean {
        val diff = abs(lift.position - target)
        return diff < threshold
    }
}