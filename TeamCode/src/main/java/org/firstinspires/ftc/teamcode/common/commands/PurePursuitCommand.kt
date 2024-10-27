package org.firstinspires.ftc.teamcode.common.commands

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandBase
import com.millburnx.util.Vec2d
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.utils.Telemetry

class PurePursuitCommand(
    val drive: Drive,
    val path: List<Vec2d>,
    val dash: FtcDashboard,
    val lookahead: Double = 14.0,
) : CommandBase() {
    //    val purePursuit = PurePursuit(path, lookahead)
    val timer: ElapsedTime = ElapsedTime()
    var loops = 0
    val fullTimer: ElapsedTime = ElapsedTime()

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
//        val calcResults = purePursuit.calc(position, heading)
//        val targetPoint = calcResults.target

//        val powerF = position.distanceTo(targetPoint)
//        val angleDiff = Util.getAngleDiff((position to heading), targetPoint)
//        val powerH = Math.toDegrees(angleDiff)

        val packet = TelemetryPacket()
        packet.put("robot/x", pose.x)
        packet.put("robot/y", pose.y)
        packet.put("robot/h", Math.toDegrees(pose.heading))
//        PurePursuit.render(calcResults, packet, true)
//        packet.put("pure_pursuit/power_forward", powerF)
//        packet.put("pure_pursuit/power_heading", powerH)
        val delta = timer.milliseconds()
        loops++
        val loopOffset = loops - 10
        val fullDelta = fullTimer.milliseconds() / if (loopOffset <= 0) 1 else loopOffset
        timer.reset()
        packet.put("general/loop_time (ms)", delta)
        packet.put("general/avg_loop_time (ms)", fullDelta)
        Telemetry.drawRobot(packet.fieldOverlay(), pose, "#0000ff")
        dash.sendTelemetryPacket(packet)

        if (isFinished) {
            drive.robotCentric(0.0, 0.0, 0.0);
            return
        }
//        drive.robotCentric(powerF, 0.0, -powerH, AutonConfig.multiF, AutonConfig.multiH)
    }

    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
    }

    override fun isFinished(): Boolean {
//        return purePursuit.isFinished
        return false
    }
}