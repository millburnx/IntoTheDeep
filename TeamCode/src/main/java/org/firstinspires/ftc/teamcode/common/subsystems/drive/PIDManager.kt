package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand.Companion.headingTolerance
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand.Companion.kD
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand.Companion.kDHeading
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand.Companion.kI
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand.Companion.kIHeading
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand.Companion.kP
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand.Companion.kPHeading
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand.Companion.usePowerSettling
import org.firstinspires.ftc.teamcode.common.commands.drive.DrivePIDCommand.Companion.wheelThreshold
import org.firstinspires.ftc.teamcode.common.utils.APIDController
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.normalizeDegrees
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonRobot
import kotlin.math.abs

class PIDManager(
    val robot: Robot,
) : Subsystem() {
    var isOn = false
    var target = Pose2d()
    var tolerance = Pose2d(DrivePIDCommand.tolerance, headingTolerance)
    val drive: Drive = robot.drive

    val pidX by lazy { PIDController(kP, kI, kD) }
    val pidY by lazy { PIDController(kP, kI, kD) }
    val pidH by lazy { APIDController(kPHeading, kIHeading, kDHeading) }

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
    val robot: AutonRobot,
    val target: Pose2d,
    val tolerance: Pose2d = Pose2d(DrivePIDCommand.tolerance, headingTolerance),
) : CommandBase() {
    init {
        addRequirements(robot.drive, robot.pidManager)
    }

    override fun execute() {
        robot.pidManager.isOn = true
        robot.pidManager.target = target
        robot.pidManager.tolerance = tolerance
    }

    override fun isFinished(): Boolean = robot.pidManager.atTarget()
}
