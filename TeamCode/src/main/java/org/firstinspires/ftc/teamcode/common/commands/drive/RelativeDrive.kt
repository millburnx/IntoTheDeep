package org.firstinspires.ftc.teamcode.common.commands.drive

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Pose2d

class RelativeDrive(
    val robot: Robot,
    val power: Pose2d,
    val useStuckDectector: Boolean = false,
) : CommandBase() {
    init {
        addRequirements(robot.drive)
    }

    override fun initialize() {
        robot.drive.pidManager.isOn = false
    }

    override fun execute() {
        robot.drive.robotCentric(-power.x, power.y, power.heading)
    }

    override fun end(interrupted: Boolean) {
        robot.drive.robotCentric(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean = useStuckDectector && robot.drive.stuckDectector.isStuck
}
