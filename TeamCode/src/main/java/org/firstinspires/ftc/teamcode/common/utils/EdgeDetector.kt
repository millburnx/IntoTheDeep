package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand

class EdgeDetector(
    val button: () -> Boolean,
    val rising: () -> Unit = {},
    val falling: () -> Unit = {},
) : Subsystem() {
    constructor(
        button: () -> Boolean,
        rising: CommandBase = InstantCommand({}),
        falling: CommandBase = InstantCommand({}),
    ) : this(
        button,
        { CommandScheduler.getInstance().schedule(rising) },
        { CommandScheduler.getInstance().schedule(falling) },
    )

    var last: Boolean = false

    override fun periodic() {
        val current = button()
        if (current && !last) {
            rising()
        } else if (!current && last) {
            falling()
        }
        last = current
    }
}
