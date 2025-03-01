package org.firstinspires.ftc.teamcode.common.commands.drive

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Pose2d

@Config
class RelativeDrive(
    val robot: Robot,
    val power: Pose2d,
    val useStuckDectector: Boolean = false,
) : CommandBase() {
    var elapsedTime = ElapsedTime()

    init {
        addRequirements(robot.drive)
    }

    override fun initialize() {
        robot.drive.pidManager.isOn = false
        elapsedTime.reset()
    }

    override fun execute() {
        robot.drive.robotCentric(-power.x, power.y, power.heading)
    }

    override fun end(interrupted: Boolean) {
        robot.drive.robotCentric(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        if (elapsedTime.milliseconds() < minStuckThreshold) return false
        return useStuckDectector && robot.drive.stuckDectector.isStuck
    }

    companion object {
        @JvmField
        var minStuckThreshold: Long = 250L
    }
}
