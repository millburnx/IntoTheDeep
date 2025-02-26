package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

@Config
class StuckDectector(
    val robot: Robot,
) : Subsystem() {
    val positionQueue = TimeQueue<Pose2d>(stuckDuration)

    val velocity: Pose2d
        get() {
            if (positionQueue.queue.size < 2) return Pose2d() // not enough data
            // sum up the abs diff, divide by time
            val deltaTime = positionQueue.queue.last().second - positionQueue.queue.first().second
            val deltas = positionQueue.queue.map { it.first }.zipWithNext()
            val travelSum = deltas.fold(Pose2d()) { acc, next -> acc + (next.first - next.second).abs() }

            return travelSum / deltaTime
        }

    val isStuck: Boolean
        get() {
            val distance = velocity.distanceTo(Vec2d())
            return distance < stuckVeloT && velocity.heading < stuckVeloH
        }

    override fun periodic() {
        robot.telemetry.addData("stuck", isStuck)
        robot.telemetry.addData("velocityX", velocity.x)
        robot.telemetry.addData("velocityY", velocity.y)
        robot.telemetry.addData("velocityH", velocity.heading)
    }

    companion object {
        @JvmField
        var stuckDuration: Long = 250L

        @JvmField
        var stuckVeloT: Double = 0.25

        @JvmField
        var stuckVeloH: Double = 0.5
    }
}

// limit based of time not elements
class TimeQueue<T>(
    var duration: Long,
) : Subsystem() {
    val elapsedTime = ElapsedTime() // timekeeper

    val queue = ArrayDeque<Pair<T, Double>>()

    override fun init() {
        elapsedTime.reset()
        queue.removeIf {
            elapsedTime.milliseconds() > it.second + duration
        }
    }
}
