package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.common.Robot

abstract class OpMode : CommandOpMode() {
    open val robot by lazy { Robot(this) }

    override fun initialize() {
        robot.init()
    }

    override fun run() {
        super.run()
        this.exec()
        robot.telemetry.update()
    }

    abstract fun exec()
}