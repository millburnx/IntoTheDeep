package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime

class DynamicWait(val durationGetter: () -> Long) : CommandBase() {
    val timer = ElapsedTime()
    var duration = Long.MAX_VALUE

    override fun initialize() {
        timer.reset()
        duration = durationGetter()
    }

    override fun isFinished(): Boolean {
        return timer.milliseconds() >= duration
    }
}