package org.firstinspires.ftc.teamcode.common.commands.drive

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.millburnx.purepursuit.PurePursuit
import com.millburnx.purepursuit.Util.getAngleDiff
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Pose2d

@Config
class PurePursuitCommand(
    val robot: Robot,
    val targetHeading: Double,
    val path: List<Vec2d>,
    val lookahead: ClosedRange<Double> = 8.0..16.0,
    val headingLock: Boolean = true, // true = heading-less pure pursuit, false = traditional pure pursuit
    val exitOnStuck: Boolean = false, // useful for if you're ramming into a wall
    val multiF: Double = Companion.multiF,
    val multiH: Double = Companion.multiH,
    val backwards: Boolean = false, // only for if headingLock = false
) : CommandBase() {
    val purePursuit = PurePursuit(path, lookahead, PIDSettings.tolerance)
    val endingState = false

    var virtualHeading: Double = 0.0

    override fun initialize() {
        virtualHeading = robot.drive.pose.heading
    }

    override fun execute() {
        val pose = robot.drive.pose
        val results =
            purePursuit.calc(
                pose.position,
                if (headingLock) virtualHeading else pose.heading,
                robot.deltaTime.deltaTime,
            )
        robot.telemetry.addData("lookahead", results.lookahead)
        robot.telemetry.addData("target", results.target)

        if (headingLock) {
            robot.telemetry.addData("virtualHeading", virtualHeading)

            virtualHeading = Math.toDegrees(pose.position.angleTo(results.target))

            robot.drive.pidManager.isOn = true
            robot.drive.pidManager.target = Pose2d(results.target, targetHeading)
        } else {
            val endDistance = pose.distanceTo(path.last())
            if (endDistance < lookahead.start) {
                robot.drive.pidManager.isOn = true
                robot.drive.pidManager.target = Pose2d(path.last(), targetHeading)
            } else {
                val powerF = pose.distanceTo(results.target) * if (backwards) -1.0 else 1.0
                val angleDiff =
                    if (backwards) {
                        getAngleDiff(pose.position to Math.toRadians(pose.degrees + 180), results.target)
                    } else {
                        getAngleDiff(pose.position to pose.radians, results.target)
                    }
                val powerH = Math.toDegrees(angleDiff)

                robot.drive.pidManager.isOn = false
                robot.drive.robotCentric(powerF * multiF, 0.0, -powerH * multiH)
            }
        }

        val packet = robot.telemetryManager.currentPacket
        PurePursuit.render(results, packet, pose.position, false)
        robot.telemetryManager.drawRobot(packet, Pose2d(pose.position, virtualHeading), "#00ff00")
    }

    override fun isFinished(): Boolean {
        if (exitOnStuck && robot.drive.stuckDectector.isStuck) return true
        return robot.drive.pidManager.atTarget()
    }

    companion object {
        @JvmField
        var multiF = 1.0

        @JvmField
        var multiH = 1.0
    }
}
