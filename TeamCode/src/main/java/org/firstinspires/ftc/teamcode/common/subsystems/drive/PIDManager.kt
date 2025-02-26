package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings.Companion.headingTolerance
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings.Companion.usePowerSettling
import org.firstinspires.ftc.teamcode.common.commands.drive.PIDSettings.Companion.wheelThreshold
import org.firstinspires.ftc.teamcode.common.utils.APIDController
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.normalizeDegrees
import kotlin.math.abs

open class PIDManager(
    val robot: Robot,
) : Subsystem() {
    var isOn = false
    open var target = Pose2d()
    var tolerance = Pose2d(PIDSettings.tolerance, headingTolerance)
    val drive: Drive = robot.drive

    val pidX by lazy { PIDController(kP, kI, kD) }
    val pidY by lazy { PIDController(kP, kI, kD) }
    val pidH by lazy { APIDController(kPHeading, kIHeading, kDHeading) }

    var kP = PIDSettings.kP
    var kI = PIDSettings.kI
    var kD = PIDSettings.kD

    var kPHeading = PIDSettings.kPHeading
    var kIHeading = PIDSettings.kIHeading
    var kDHeading = PIDSettings.kDHeading

    override fun periodic() {
        if (!isOn) return
        pidX.setPID(kP, kI, kD)
        pidY.setPID(kP, kI, kD)
        pidH.setPID(kPHeading, kIHeading, kDHeading)

        val x = pidX.calculate(drive.pose.x, target.x)
        val y = pidY.calculate(drive.pose.y, target.y)
        val h = pidH.calculate(drive.pose.heading, target.heading)

        drive.fieldCentric(
            -x,
            y,
            h,
            -Math.toRadians(drive.pose.heading),
        )
    }

    fun atTarget(): Boolean {
        if (!isOn) return true
        val diff = (target - drive.pose).abs()
        val atX = diff.x < tolerance.x
        val atY = diff.y < tolerance.y
        val atH = normalizeDegrees(diff.heading) < tolerance.heading
//        println("atX: $atX, atY: $atY, atH: $atH $diff ${drive.pose} $target")

        val motorThreshold = drive.motors.all { abs(it.power) < wheelThreshold }

        println("$motorThreshold ${drive.motors.map { abs(it.power) }}")

        if (!motorThreshold && usePowerSettling) return false

        return atX && atY && atH
    }
}

class PIDCommand(
    val robot: Robot,
    val target: Pose2d,
    val tolerance: Pose2d = Pose2d(PIDSettings.tolerance, headingTolerance),
) : CommandBase() {
    init {
        addRequirements(robot.drive, robot.drive.pidManager)
    }

    override fun execute() {
        robot.drive.pidManager.isOn = true
        robot.drive.pidManager.target = target
        robot.drive.pidManager.tolerance = tolerance
    }

    override fun isFinished(): Boolean = robot.drive.pidManager.atTarget()
}
