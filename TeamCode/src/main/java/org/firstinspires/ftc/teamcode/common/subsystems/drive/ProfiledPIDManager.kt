package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.normalizeDegrees
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

class ProfiledPIDManager(
    robot: Robot,
) : PIDManager(robot) {
    // this is NOT a traditional motion profile as it is positional based
    // but has code to prevent oscillations if the robot happens to be faster than the desired profile

    val currentTransVelocity
        get() = (robot.drive.oldPose.distanceTo(robot.drive.pose)) / robot.deltaTime.deltaTime
    val currentHeadingVelocity
        get() = normalizeDegrees(robot.drive.pose.heading - robot.drive.oldPose.heading) / robot.deltaTime.deltaTime

    var profiledTarget = Pose2d()

    var sign = Pose2d()

    var isDecelerating = false

    override var target = Pose2d()
        set(value) {
            field = value
            profiledTarget = robot.drive.pose // reset
            sign = (value - robot.drive.pose).sign()
        }

    override fun periodic() {
        if (!isOn) return

        // check what state we should be in
        if (!isDecelerating && shouldDecelerate()) {
            isDecelerating = true
            // reset for deceleration, otherwise it will be already at target (then no deceleration happens)
            profiledTarget = robot.drive.pose
        }

        if (!isDecelerating) {
            val newVelocity = currentTransVelocity + maxAccelTrans
            val forwardMost = forwardMost()
            val directionVector = target.position - forwardMost
            val correctedVector = directionVector.normalize() * newVelocity // rescale to target velocity
            val newPose = clamp(forwardMost + correctedVector, forwardMost)

            val newHeadingVelocity = currentHeadingVelocity + maxAccelHeading
            val newHeading = normalizeDegrees(robot.drive.pose.heading + newHeadingVelocity)
            profiledTarget = Pose2d(newPose, newHeading)
        } else {
            val newVelocity = currentTransVelocity - maxAccelTrans
            // we might not need forwardMost for deceleration tbh, can prob just use the bot pose
            val forwardMost = forwardMost()
            val directionVector = target.position - forwardMost
            val correctedVector = directionVector.normalize() * newVelocity // rescale to target velocity
            val newPose = clamp(forwardMost + correctedVector, forwardMost)

            val newHeadingVelocity = currentHeadingVelocity - maxAccelHeading
            val newHeading = normalizeDegrees(robot.drive.pose.heading + newHeadingVelocity)
            profiledTarget = Pose2d(newPose, newHeading)
        }

        // actual PID part, taken from the base PIDManager
        pidX.setPID(kP, kI, kD)
        pidY.setPID(kP, kI, kD)
        pidH.setPID(kPHeading, kIHeading, kDHeading)

        val x = pidX.calculate(robot.drive.pose.x, profiledTarget.x)
        val y = pidY.calculate(robot.drive.pose.y, profiledTarget.y)
        val h = pidH.calculate(robot.drive.pose.heading, profiledTarget.heading)

        robot.drive.fieldCentric(-x, y, h, -robot.drive.pose.radians)
    }

    // prevents overshooting the target or going backwards
    fun clamp(
        pos: Vec2d,
        forwardMost: Vec2d,
    ): Vec2d {
        // uses sign instead of min/max to make sure once we pass the target, it gets stuck there
        val minX = if (sign.x > 0) forwardMost.x else target.x
        val maxX = if (sign.x > 0) target.x else forwardMost.x
        val minY = if (sign.y > 0) forwardMost.y else target.y
        val maxY = if (sign.y > 0) target.y else forwardMost.y

        val x = pos.x.clamp(minX, maxX)
        val y = pos.y.clamp(minY, maxY)
        return Vec2d(x, y)
    }

    // closest point to target taken from the profiled target & current pose
    // basically prevents the robot from going backwards if its already past the profiled target
    fun forwardMost(): Vec2d {
        val x = if (sign.x > 0) max(profiledTarget.x, robot.drive.pose.x) else min(profiledTarget.x, robot.drive.pose.x)
        val y = if (sign.y > 0) max(profiledTarget.y, robot.drive.pose.y) else min(profiledTarget.y, robot.drive.pose.y)
        return Vec2d(x, y)
    }

    // https://www.desmos.com/calculator/q5xaa1mdyc
    fun shouldDecelerate(): Boolean {
        val timeForDecleration = currentTransVelocity / maxAccelTrans
        val distanceForDeceleration =
            currentTransVelocity * timeForDecleration + (-maxAccelTrans * timeForDecleration.pow(2)) / 2
        val distanceToTarget = robot.drive.pose.distanceTo(target)
        return distanceToTarget < distanceForDeceleration
        return false
    }

    companion object {
        @JvmField
        var maxAccelTrans = 48.0 // in random ass units, just trial and error it lol

        @JvmField
        var maxAccelHeading = 180.0
    }
}
