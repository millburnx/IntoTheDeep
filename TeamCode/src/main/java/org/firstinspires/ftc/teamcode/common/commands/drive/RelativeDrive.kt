package org.firstinspires.ftc.teamcode.common.commands.drive

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDManager
import org.firstinspires.ftc.teamcode.common.utils.Pose2d

class RelativeDrive(val drive: Drive, val pidManager: PIDManager, val power: Pose2d) : CommandBase() {
    init {
        addRequirements(drive)
    }

    override fun initialize() {
        pidManager.isOn = false
    }

    override fun execute() {
        drive.robotCentric(power.x, power.y, power.heading)
    }

    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        // we should add smt w/ odom to seem if we are moving at all
        // if we are fully still for a while then we're already hitting the wall
        return false
    }
}