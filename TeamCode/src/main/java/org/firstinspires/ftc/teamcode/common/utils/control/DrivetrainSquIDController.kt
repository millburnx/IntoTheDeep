package org.firstinspires.ftc.teamcode.common.utils.control

import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.utils.DeltaTime
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sqrt

@Config
class DrivetrainSquIDController(
    var p: Double,
    val deltaTime: DeltaTime,
) {
    fun calculate(
        targetPose: Vec2d,
        currentPose: Pose2d,
        currentVelocity: Pose2d,
    ): Vec2d {
        var currentPose = currentPose
        var currentVelMag = currentVelocity.position.normalize()
        // compensate for pinpoint trolling
        val deltaSeconds = deltaTime.deltaTime
        currentVelMag *= deltaSeconds

        val velAngle = atan2(currentVelocity.y, currentVelocity.x)

        val distance = max(getDistanceFromVelocity(currentVelMag), 0.0)

        currentPose += Vec2d.fromAngle(velAngle) * distance

        var magnitude = currentPose.distanceTo(targetPose)
        magnitude = sqrt(abs((magnitude - 0) * p)) * sign(magnitude - 0)
        val delta = targetPose - currentPose.position
        val posAngle = atan2(delta.y, delta.x)
        return Vec2d.fromAngle(posAngle) * magnitude
    }

    companion object {
        var looptimeAdjuster: Double = 15.0

        private fun getDistanceFromVelocity(velocity: Double): Double {
            var velocity = velocity
            velocity *= looptimeAdjuster
            // equation from regression
            return 0.00286 * velocity * velocity + 0.304 * velocity - 0.837
        }
    }
}
