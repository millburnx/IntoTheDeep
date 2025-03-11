package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings.Companion.headingTolerance
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings.Companion.usePowerSettling
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings.Companion.wheelThreshold
import org.firstinspires.ftc.teamcode.common.commands.drive.PickupPIDSettings
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.control.APIDController
import org.firstinspires.ftc.teamcode.common.utils.control.ASQUIDController
import org.firstinspires.ftc.teamcode.common.utils.control.SQUIDController
import org.firstinspires.ftc.teamcode.common.utils.normalizeDegrees
import kotlin.math.abs

@Config
open class PIDManager(
    val robot: Robot,
) : Subsystem() {
    var isOn = false
        set(value) {
            if (field != value) {
                field = value
                if (value) {
                    robot.drive.breakMotors()
                    isTeleopHolding = false
                } else {
                    robot.drive.floatMotors()
                    isTeleopHolding = true // default to hold
                }
            }
        }

    var isTeleopHolding = false
        set(value) {
            if (field != value) {
                field = value
                if (value) robot.drive.breakMotors() else robot.drive.floatMotors()
            }
        }

    open var target = Pose2d()
    var tolerance = Pose2d(PIDSettings.tolerance, headingTolerance)

    val pidX by lazy { PIDController(kP, kI, kD) }
    val pidY by lazy { PIDController(kP, kI, kD) }
    val pidH by lazy { APIDController(kPHeading, kIHeading, kDHeading) }

    val squidX by lazy { SQUIDController(kP, kI, kD) }
    val squidY by lazy { SQUIDController(kP, kI, kD) }
    val squidH by lazy { ASQUIDController(kPHeading, kIHeading, kDHeading) }

    var isSamplePickup: Boolean = false
        set(value) {
            field = value
            tolerance =
                if (value) {
                    Pose2d(PickupPIDSettings.tolerance, PickupPIDSettings.headingTolerance)
                } else {
                    Pose2d(
                        PIDSettings.tolerance,
                        headingTolerance,
                    )
                }
        }

    val kP
        get() = if (isSamplePickup) PickupPIDSettings.kP else PIDSettings.kP
    val kI
        get() = if (isSamplePickup) PickupPIDSettings.kI else PIDSettings.kI
    val kD
        get() = if (isSamplePickup) PickupPIDSettings.kD else PIDSettings.kD

    val kPHeading
        get() = if (isSamplePickup) PickupPIDSettings.kPHeading else PIDSettings.kPHeading
    val kIHeading
        get() = if (isSamplePickup) PickupPIDSettings.kIHeading else PIDSettings.kIHeading
    val kDHeading
        get() = if (isSamplePickup) PickupPIDSettings.kDHeading else PIDSettings.kDHeading

    override fun periodic() {
        if (!isOn && !isTeleopHolding) return
        pidX.setPID(kP, kI, kD)
        pidY.setPID(kP, kI, kD)
        pidH.setPID(kPHeading, kIHeading, kDHeading)
        squidX.setPID(kP, kI, kD)
        squidY.setPID(kP, kI, kD)
        squidH.setPID(kPHeading, kIHeading, kDHeading)

        var target = target

        if (coastStop && dragCoefficient != 0.0) {
            val velo = robot.drive.velocity.position

            val stoppingDistance = velo.magnituide() / dragCoefficient

            val delta = velo / velo.magnituide() * stoppingDistance
            val stop = robot.drive.pose.position + delta
            val diff = target.position - stop
            target = robot.drive.pose + diff // cut power except for corrective
        }

        val x = pidX.calculate(robot.drive.pose.x, target.x)
        val y = pidY.calculate(robot.drive.pose.y, target.y)
        val h = pidH.calculate(robot.drive.pose.heading, target.heading)

        val squidX = pidX.calculate(robot.drive.pose.x, target.x)
        val squidY = pidY.calculate(robot.drive.pose.y, target.y)
        val squidH = pidH.calculate(robot.drive.pose.heading, target.heading)

//        robot.drive.fieldCentric(-x, y, h, -robot.drive.pose.radians)
        robot.drive.fieldCentric(
            x = if (PIDSettings.squid) -squidX else -x,
            y = if (PIDSettings.squid) squidY else y,
            rotate = if (PIDSettings.squid) squidH else h,
            -robot.drive.pose.radians,
        )
    }

    fun atTarget(): Boolean {
        if (!isOn) return true
        val diff = (target - robot.drive.pose).abs()
        val atX = diff.x < tolerance.x
        val atY = diff.y < tolerance.y
        val atH = normalizeDegrees(diff.heading) < tolerance.heading
//        println("atX: $atX, atY: $atY, atH: $atH $diff ${drive.pose} $target")

        val motorThreshold = robot.drive.motors.all { abs(it.power) < wheelThreshold }

        println("$motorThreshold ${robot.drive.motors.map { abs(it.power) }}")

        if (!motorThreshold && usePowerSettling) return false

        return atX && atY && atH
    }

    companion object {
        @JvmField
        var coastStop = false

        @JvmField
        var dragCoefficient = 0.0
    }
}

class PIDCommand(
    val robot: Robot,
    val target: Pose2d,
    val tolerance: Pose2d = Pose2d(PIDSettings.tolerance, headingTolerance),
    val useStuckDectector: Boolean = false,
) : CommandBase() {
    val elapsedTime = ElapsedTime()

    init {
        addRequirements(robot.drive, robot.drive.pidManager)
    }

    override fun initialize() {
        elapsedTime.reset()
    }

    override fun execute() {
        robot.drive.pidManager.isOn = true
        robot.drive.pidManager.target = target
        robot.drive.pidManager.tolerance = tolerance
    }

    override fun isFinished(): Boolean {
        if (useStuckDectector && elapsedTime.milliseconds() > minStuckThreshold && robot.drive.stuckDectector.isStuck) return true
        return robot.drive.pidManager.atTarget()
    }

    companion object {
        @JvmField
        var minStuckThreshold: Long = 250L
    }
}
