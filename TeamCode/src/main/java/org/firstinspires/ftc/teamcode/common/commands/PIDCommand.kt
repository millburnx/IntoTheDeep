package org.firstinspires.ftc.teamcode.common.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.commands.PurePursuitCommand.Companion.useSquidRotation
import org.firstinspires.ftc.teamcode.common.commands.PurePursuitCommand.Companion.useSquidTranslation
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonConfig
import org.firstinspires.ftc.teamcode.opmodes.tuning.APIDController
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

class PIDCommand(
    val drive: Drive,
    val target: Vec2d,
    val targetHeading: Double? = null,
    val pidX: PIDController = PIDController(0.0, 0.0, 0.0),
    val pidY: PIDController = PIDController(0.0, 0.0, 0.0),
    val pidH: APIDController = APIDController(0.0, 0.0, 0.0),
    val multiF: Double = 1.0,
    val multiH: Double = 1.0,
    val threshold: Double = AutonConfig.threshold,
    val thresholdHeading: Double = AutonConfig.headingTolerance,
) : CommandBase() {
    var hasReset: Boolean = false

    init {
        addRequirements(drive)
    }

    override fun execute() {
        if (!hasReset) {
            println("RESETTING PID")
            hasReset = true
            pidX.reset()
            pidY.reset()
            pidH.reset()
        }

        val pose = drive.pose
        val position = Vec2d(pose.x, pose.y)

        val targetPoint = target
        val powerX = pidX.calculate(position.x, targetPoint.x)
        val finalX = if (useSquidTranslation) sqrt(abs(powerX)) * sign(powerX) else powerX
        val powerY = -pidY.calculate(position.y, targetPoint.y)
        val finalY = if (useSquidTranslation) sqrt(abs(powerY)) * sign(powerY) else powerY
        val powerH = -pidH.calculate(Math.toDegrees(pose.heading), targetHeading ?: pose.heading)
        val finalH = if (useSquidRotation) sqrt(abs(powerH)) * sign(powerH) else powerH

        drive.fieldCentric(finalX, finalY, finalH, pose.heading, multiF, multiH)
    }

    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        val pose = drive.pose
        val distance = Vec2d(pose.x, pose.y).distanceTo(target)
        val isTranslation = distance < threshold
        if (targetHeading == null) return isTranslation
        val diffHeading = abs(Math.toDegrees(drive.pose.heading) - targetHeading)
        val isHeading = diffHeading < thresholdHeading
        return isHeading && isTranslation
    }
}