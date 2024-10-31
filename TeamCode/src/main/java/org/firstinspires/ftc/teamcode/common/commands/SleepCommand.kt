package org.firstinspires.ftc.teamcode.common.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime

class SleepCommand(val ms: Double) : CommandBase() {
    val timer = ElapsedTime()
    var hasReset = false

    override fun execute() {
        if (!hasReset) {
            timer.reset()
            hasReset = true
        }
    }

    override fun isFinished(): Boolean {
        return timer.milliseconds() > ms
    }
}