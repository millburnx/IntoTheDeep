package org.firstinspires.ftc.teamcode.common.subsystems.drive

import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Pose2d

class ProfiledPIDManager(
    robot: Robot,
) : PIDManager(robot) {
    // this is NOT a traditional motion profile as it is positional based
    // but has code to prevent oscillations if the robot happens to be faster than the desired profile

//    override var profiledTarget

    override var target = Pose2d()
}
