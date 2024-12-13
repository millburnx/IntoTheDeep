package org.firstinspires.ftc.teamcode.common.commands.drive

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import kotlin.math.absoluteValue

@Config
class DrivePIDCommand(
    val drive: Drive,
    val target: Pose2d,
    val tolerance: Double = Companion.tolerance,
    val headingTolerance: Double = Companion.headingTolerance
) : CommandBase() {
    val pidX by lazy { PIDController(kP, kI, kD) }
    val pidY by lazy { PIDController(kP, kI, kD) }
    val pidH by lazy { PIDController(kPHeading, kIHeading, kDHeading) }

    init {
        addRequirements(drive)
    }

    override fun execute() {
        pidX.setPID(kP, kI, kD)
        pidY.setPID(kP, kI, kD)
        pidH.setPID(kPHeading, kIHeading, kDHeading)

        val x = pidX.calculate(drive.pose.x, target.x)
        val y = pidY.calculate(drive.pose.y, target.y)
        val h = pidH.calculate(drive.pose.heading, target.heading)

        drive.fieldCentric(x, y, h, drive.pose.heading)
    }

    override fun isFinished(): Boolean {
        val diff = target.position - drive.pose.position
        val headingDiff = target.heading - drive.pose.heading
        val atX = diff.x.absoluteValue < tolerance
        val atY = diff.y.absoluteValue < tolerance
        val atH = headingDiff.absoluteValue < headingTolerance
        return atX && atY && atH
    }

    companion object {
        @JvmField
        var kP: Double = 0.0

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kPHeading: Double = 0.0

        @JvmField
        var kIHeading: Double = 0.0

        @JvmField
        var kDHeading: Double = 0.0

        @JvmField
        var tolerance: Double = 1.0

        @JvmField
        var headingTolerance: Double = 1.0
    }
}