package org.firstinspires.ftc.teamcode.common.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import kotlin.math.abs

class LiftCommand(val lift: Lift, val targetFun: () -> Double, val threshold: Int = 50) : CommandBase() {
    constructor(lift: Lift, targetFun: Double, threshold: Int = 50) : this(lift, { targetFun }, threshold)

    val target: Double by lazy { targetFun() }

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
        val diff = abs(lift.position - targetFun())
        return diff < threshold
    }
}