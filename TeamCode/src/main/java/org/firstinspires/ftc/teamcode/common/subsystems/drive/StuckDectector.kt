package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class StuckDectector(
    val robot: Robot,
) : Subsystem() {
    val positionQueue = TimeQueue<Pose2d>(stuckDuration)

    val velocity: Pose2d
        get() {
            // sum up the abs diff, divide by time
            val veloSum = positionQueue.queue
            return Pose2d()
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
