package org.firstinspires.ftc.teamcode.common.commands.drive

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.millburnx.purepursuit.PurePursuit
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonRobot

@Config
class PurePursuitCommand(
    val robot: AutonRobot,
    val targetHeading: Double,
    val path: List<Vec2d>,
    val lookahead: ClosedRange<Double> = 8.0..16.0,
) : CommandBase() {
    val purePursuit = PurePursuit(path, lookahead, PIDSettings.tolerance)
    val endingState = false

    var virtualHeading: Double = 0.0

    override fun initialize() {
        virtualHeading = robot.drive.pose.heading
    }

    override fun execute() {
        val pose = robot.drive.pose
        val results = purePursuit.calc(pose.position, virtualHeading, robot.deltaTime.deltaTime)
        robot.telemetry.addData("lookahead", results.lookahead)
        robot.telemetry.addData("virtualHeading", virtualHeading)
        robot.telemetry.addData("target", results.target)

        virtualHeading = Math.toDegrees(pose.position.angleTo(results.target))

        robot.pidManager.isOn = true
        robot.pidManager.target = Pose2d(results.target, targetHeading)

        val packet = robot.telemetryManager.currentPacket
        PurePursuit.render(results, packet, pose.position, false)
        robot.telemetryManager.drawRobot(packet, Pose2d(pose.position, virtualHeading), "#00ff00")
    }

    override fun isFinished(): Boolean = robot.pidManager.atTarget()

    companion object {
        @JvmField
        var virtualHeadingSpeed = 1.0

        @JvmField
        var maxVirtualHeadingSpeed = 90.0
    }
}
