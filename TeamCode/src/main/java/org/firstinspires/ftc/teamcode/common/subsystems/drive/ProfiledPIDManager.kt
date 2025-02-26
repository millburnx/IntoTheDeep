package org.firstinspires.ftc.teamcode.common.subsystems.drive

import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Pose2d

class ProfiledPIDManager(
    robot: Robot,
) : PIDManager(robot) {
    // this is NOT a traditional motion profile as it is positional based
    // but has code to prevent oscillations if the robot happens to be faster than the desired profile

    var profiledTarget = Pose2d()

    var isDecelerating = false

    override var target = Pose2d()
        set(value) {
            field = value
            profiledTarget = robot.drive.pose // reset
        }

    override fun periodic() {
        // check what state we should be in
        if (!isDecelerating && shouldDecelerate()) isDecelerating = true
    }

    fun shouldDecelerate(): Boolean {
//        val currentVelocity = (robot.drive.oldPose.distanceTo(robot.drive.pose)) / robot.deltaTime.deltaTime
//        val distanceForDeceleration = sqrt((currentVelocity * 2) / -maxAccelTrans)
//        val distanceToTarget = robot.drive.pose.distanceTo(target)
//        return distanceToTarget < distanceForDeceleration
        return false
    }

    companion object {
        @JvmField
        var maxAccelTrans = 48.0

        @JvmField
        var maxAccelHeading = 180.0
    }
}
