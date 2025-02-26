package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.common.Robot

abstract class OpMode : CommandOpMode() {
    open val robot by lazy { Robot(this) }

    override fun initialize() {
        robot.init()
    }

    override fun run() {
        for (hub in robot.hubs) {
            hub.clearBulkCache()
        }
        robot.telemetryManager.pre()
        super.run()
        this.exec()
        robot.telemetryManager.post()
    }

    abstract fun exec()
}
