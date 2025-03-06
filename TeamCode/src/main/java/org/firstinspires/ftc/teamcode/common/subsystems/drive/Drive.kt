package org.firstinspires.ftc.teamcode.common.subsystems.drive

import android.os.Environment
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.millburnx.utils.Path
import com.millburnx.utils.TSV
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings.Companion.headingTolerance
import org.firstinspires.ftc.teamcode.common.commands.drive.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.commands.drive.RelativeDrive
import org.firstinspires.ftc.teamcode.common.utils.CachedMotor
import org.firstinspires.ftc.teamcode.common.utils.PinPoint
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.loadPath
import org.firstinspires.ftc.teamcode.common.utils.reset
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
    open val subsystems: List<Subsystem> = listOf(pidManager, stuckDectector)

    val frontLeft = CachedMotor(robot.hardware, "frontLeft", cacheThreshold, breakMotors)
    val frontRight = CachedMotor(robot.hardware, "frontRight", cacheThreshold, breakMotors, false)
    val backLeft = CachedMotor(robot.hardware, "backLeft", cacheThreshold, breakMotors)
    val backRight = CachedMotor(robot.hardware, "backRight", cacheThreshold, breakMotors, false)

    val motors = listOf(frontLeft, frontRight, backLeft, backRight)

    val pinPoint by lazy { PinPoint(robot.hardware, "pinpoint") }
    var pose: Pose2d
        get() = pinPoint.pose
        set(value) {
            pinPoint.pose = value
        }

    val velocity: Pose2d
        get() = pinPoint.velocity

    var oldPose: Pose2d = Pose2d()

    override fun init() {
        (robot.hardware["para"] as DcMotorEx).reset()
        (robot.hardware["frontLeft"] as DcMotorEx).reset()
        subsystems.forEach { it.init() }
    }

    fun stop() =
        InstantCommand({
            robot.drive.robotCentric(0.0, 0.0, 0.0)
        })

    fun breakMotors() {
        motors.forEach { it.isBrake = true }
    }

    fun floatMotors() {
        motors.forEach { it.isBrake = false }
    }

    override fun periodic() {
        oldPose = pose
        pinPoint.update()
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
        heading: Double = pose.radians,
    ) {
        val relativeVector = Vec2d(x, y).rotate(-heading) * Vec2d(1.0, strafeMultiplier)

        val forward = relativeVector.x
        val strafe = relativeVector.y

        val weightedRotate = rotate + strafe * frontWeighting + strafe * extendoWeighting * robot.intake.linkage.target

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
        useStuckDectector: Boolean = false,
    ) = PIDCommand(robot, target, tolerance, useStuckDectector)

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
        useStuckDectector: Boolean = false,
    ) = purePursuit(Path.loadPath(path), heading, useStuckDectector)

    fun purePursuit(
        path: Path,
        heading: Double,
        useStuckDectector: Boolean = false,
    ) = PurePursuitCommand(robot, heading, path.points, exitOnStuck = useStuckDectector)

    companion object {
        @JvmField
        var strafeMultiplier: Double = 1.1

        @JvmField
        var frontWeighting: Double = 0.0

        @JvmField
        var extendoWeighting: Double = 0.0

        @JvmField
        var cacheThreshold: Double = 0.0
    }
}
