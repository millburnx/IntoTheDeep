package org.firstinspires.ftc.teamcode.common.subsystems.drive

import android.os.Environment
import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Path
import com.millburnx.utils.TSV
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings.Companion.headingTolerance
import org.firstinspires.ftc.teamcode.common.commands.drive.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.commands.drive.RelativeDrive
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.init
import org.firstinspires.ftc.teamcode.common.utils.loadPath
import org.firstinspires.ftc.teamcode.common.utils.reset
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive
import java.io.File
import kotlin.math.absoluteValue
import kotlin.math.max

@Config
open class Drive(
    val robot: Robot,
    breakMotors: Boolean = false,
) : Subsystem() {
    open val pidManager: PIDManager = PIDManager(robot)
    val stuckDectector = StuckDectector(robot)
    val subsystems: List<Subsystem> = listOf(pidManager, stuckDectector)

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
            oldPose = value
        }

    var oldPose: Pose2d = Pose2d()

    override fun init() {
        (robot.hardware["para"] as DcMotorEx).reset()
        (robot.hardware["frontLeft"] as DcMotorEx).reset()
        subsystems.forEach { it.init() }
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
        oldPose = pose
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

        val weightedRotate = rotate + strafe * extendoWeighting * robot.intake.linkage.target

        val denominator = max(forward.absoluteValue + strafe.absoluteValue + weightedRotate.absoluteValue, 1.0)
        frontLeft.power = (forward + strafe + weightedRotate) / denominator
        backLeft.power = (forward - strafe + weightedRotate) / denominator
        frontRight.power = (forward - strafe - weightedRotate) / denominator
        backRight.power = (forward + strafe - weightedRotate) / denominator
    }

    fun relativeDrive(
        power: Pose2d,
        useStuckDectector: Boolean = false,
    ) = RelativeDrive(robot, power, useStuckDectector)

    fun pid(
        target: Pose2d,
        tolerance: Pose2d = Pose2d(PIDSettings.tolerance, headingTolerance),
    ) = PIDCommand(robot, target, tolerance)

    fun pointsToPid(file: String): List<PIDCommand> {
        val csv = TSV.bufferedRead(File("${Environment.getExternalStorageDirectory().path}/Paths/$file.tsv"))
        val points: MutableList<Pose2d> = mutableListOf()
        for (item in csv) {
            points.add(Pose2d(item[0].toDouble(), item[1].toDouble(), item[2].toDouble()))
        }
        return pointsToPid(points)
    }

    fun pointsToPid(points: List<Pose2d>): List<PIDCommand> = points.map { robot.drive.pid(it) }

    fun purePursuit(
        path: String,
        heading: Double,
    ) = purePursuit(Path.loadPath(path), heading)

    fun purePursuit(
        path: Path,
        heading: Double,
    ) = PurePursuitCommand(robot, heading, path.points)

    companion object {
        @JvmField
        var strafeMultiplier: Double = 1.1

        @JvmField
        var extendoWeighting: Double = 0.0
    }
}
