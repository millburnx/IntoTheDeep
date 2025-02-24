package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init
import org.firstinspires.ftc.teamcode.common.utils.reset
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive
import kotlin.math.absoluteValue
import kotlin.math.max

@Config
open class Drive(
    val robot: Robot,
    breakMotors: Boolean = false,
) : Subsystem() {
    val frontLeft: DcMotorEx = (robot.hardware["frontLeft"] as DcMotorEx).apply { init(isBrake = breakMotors) }
    val frontRight: DcMotorEx = (robot.hardware["frontRight"] as DcMotorEx).apply { init(false, isBrake = breakMotors) }
    val backLeft: DcMotorEx = (robot.hardware["backLeft"] as DcMotorEx).apply { init(isBrake = breakMotors) }
    val backRight: DcMotorEx = (robot.hardware["backRight"] as DcMotorEx).apply { init(false, isBrake = breakMotors) }
    val motors = listOf(frontLeft, frontRight, backLeft, backRight)
    val odometry = SampleMecanumDrive(robot.hardware).localizer
    var pose: Pose2d
        get() {
            return Pose2d.fromRR(odometry.poseEstimate)
        }
        set(value) {
            odometry.poseEstimate = value.toRawRR()
        }

    override fun init() {
        super.init()
        (robot.hardware["para"] as DcMotorEx).reset()
        (robot.hardware["frontLeft"] as DcMotorEx).reset()
    }

    fun breakMotors() {
        listOf(frontLeft, frontRight, backLeft, backRight).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    fun floatMotors() {
        listOf(frontLeft, frontRight, backLeft, backRight).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }
    }

    override fun periodic() {
        super.periodic()
        odometry.update()
    }

    fun robotCentric(
        forward: Double,
        strafe: Double,
        rotate: Double,
    ) = fieldCentric(forward, strafe, rotate, 0.0)

    open fun fieldCentric(
        x: Double,
        y: Double,
        rotate: Double,
        heading: Double,
    ) {
        val relativeVector = Vec2d(x, y).rotate(-heading) * Vec2d(1.0, strafeMultiplier)

        val forward = relativeVector.x
        val strafe = relativeVector.y

        val denominator = max(forward.absoluteValue + strafe.absoluteValue + rotate.absoluteValue, 1.0)
        frontLeft.power = (forward + strafe + rotate) / denominator
        backLeft.power = (forward - strafe + rotate) / denominator
        frontRight.power = (forward - strafe - rotate) / denominator
        backRight.power = (forward + strafe - rotate) / denominator
    }

    companion object {
        @JvmField
        var strafeMultiplier: Double = 1.1
    }
}
