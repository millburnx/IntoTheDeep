package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive
import kotlin.math.absoluteValue
import kotlin.math.max

open class Drive(robot: Robot) : Subsystem() {
    val frontLeft: DcMotorEx = (robot.hardware["frontLeft"] as DcMotorEx).also { it.init() }
    val frontRight: DcMotorEx = (robot.hardware["frontRight"] as DcMotorEx).also { it.init() }
    val backLeft: DcMotorEx = (robot.hardware["backLeft"] as DcMotorEx).also { it.init() }
    val backRight: DcMotorEx = (robot.hardware["backRight"] as DcMotorEx).also { it.init() }
    val motors = listOf<DcMotorEx>(frontLeft, frontRight, backLeft, backRight)
    val odometry = SampleMecanumDrive(robot.hardware).localizer

    fun robotCentric(forward: Double, strafe: Double, rotate: Double) = fieldCentric(forward, strafe, rotate, 0.0)

    open fun fieldCentric(x: Double, y: Double, rotate: Double, heading: Double) {
        val relativeVector = Vec2d(x, y).rotate(-heading) * Vec2d(1.0, 1.1)

        val forward = relativeVector.x
        val strafe = relativeVector.y

        val denominator = max(forward.absoluteValue + strafe.absoluteValue + rotate.absoluteValue, 1.0)
        frontLeft.power = (forward + strafe + rotate) / denominator
        frontRight.power = (forward - strafe + rotate) / denominator
        backLeft.power = (forward - strafe - rotate) / denominator
        backRight.power = (forward + strafe - rotate) / denominator
    }
}