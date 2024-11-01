package org.firstinspires.ftc.teamcode.common.commands

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.purepursuit.PurePursuit
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import org.firstinspires.ftc.teamcode.common.utils.Util
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonConfig
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

@Config
class PurePursuitCommand(
    val drive: Drive,
    val path: List<Vec2d>,
    val endingHeading: Double? = null,
    val dash: FtcDashboard,
    val pidX: PIDController = PIDController(0.0, 0.0, 0.0),
    val pidY: PIDController = PIDController(0.0, 0.0, 0.0),
    val pidH: PIDController = PIDController(0.0, 0.0, 0.0),
    val lookahead: ClosedFloatingPointRange<Double> = 8.0..16.0,
) : CommandBase() {
    companion object {
        @JvmField
        var useSquidTranslation = false

        @JvmField
        var useSquidRotation = false
    }

    val purePursuit = PurePursuit(path, lookahead)
    val timer: ElapsedTime = ElapsedTime()
    var loops = 0
    val fullTimer: ElapsedTime = ElapsedTime()
    var lastEndingState = false

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        timer.reset()
        fullTimer.reset()
    }

    override fun execute() {
        if (loops == 10) fullTimer.reset()
        val pose = drive.pose
        val position = Vec2d(pose.x, pose.y)
        val heading = pose.heading
        val calcResults = purePursuit.calc(position, heading, timer.seconds())

        val packet = TelemetryPacket()
        packet.put("robot/x", pose.x)
        packet.put("robot/y", pose.y)
        packet.put("robot/h", Math.toDegrees(pose.heading))
        packet.put("robot/lookahead", calcResults.lookahead)
        PurePursuit.render(calcResults, packet, Vec2d(pose.x, pose.y))

        if (calcResults.isFinished) {
            packet.put("pure_pursuit/power_forward", 0.0)
            packet.put("pure_pursuit/power_heading", 0.0)
            drive.robotCentric(0.0, 0.0, 0.0);
        } else if (calcResults.isEnding && endingHeading != null) {
            // pid finishing move
            if (!lastEndingState) {
                pidX.reset()
                pidY.reset()
                pidH.reset()
            }
            // just p2p
            val targetPoint = calcResults.target
            val powerX = pidX.calculate(position.x, targetPoint.x)
            val finalX = if (useSquidTranslation) sqrt(abs(powerX)) * sign(powerX) else powerX
            val powerY = pidY.calculate(position.y, targetPoint.y)
            val finalY = if (useSquidTranslation) sqrt(abs(powerY)) * sign(powerY) else powerY
            val powerH = pidH.calculate(Math.toDegrees(heading), endingHeading)
            val finalH = if (useSquidRotation) sqrt(abs(powerH)) * sign(powerH) else powerH
            packet.put("pure_pursuit/power_x", finalX)
            packet.put("pure_pursuit/power_y", finalY)
            packet.put("pure_pursuit/power_heading", finalH)
            drive.fieldCentric(finalX, finalY, finalH, pose.heading)
        } else {
            val targetPoint = calcResults.target
            val powerF = position.distanceTo(targetPoint)
            val angleDiff = Util.getAngleDiff((position to heading), targetPoint)
            val powerH = Math.toDegrees(angleDiff)

            packet.put("pure_pursuit/power_forward", powerF)
            packet.put("pure_pursuit/power_heading", powerH)
            drive.robotCentric(powerF, 0.0, -powerH, AutonConfig.multiF, AutonConfig.multiH)
        }
        lastEndingState = calcResults.isEnding

        val delta = timer.milliseconds()
        loops++
        val loopOffset = loops - 10
        val fullDelta = fullTimer.milliseconds() / if (loopOffset <= 0) 1 else loopOffset
        timer.reset()
        packet.put("general/loop_time (ms)", delta)
        packet.put("general/avg_loop_time (ms)", fullDelta)
        Telemetry.drawRobot(packet.fieldOverlay(), pose, "#0000ff")
        dash.sendTelemetryPacket(packet)


    }

    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        if (endingHeading == null) return purePursuit.isFinished
        val diffHeading = abs(drive.pose.heading - endingHeading)
        return purePursuit.isFinished && diffHeading < AutonConfig.headingTolerance
    }
}