package org.firstinspires.ftc.teamcode.common.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.Robot

@Config
class TelemetryManager(
    val robot: Robot,
) {
    val dashboard: FtcDashboard = FtcDashboard.getInstance()
    var currentPacket: TelemetryPacket = TelemetryPacket()
    val telemetry = MultipleTelemetry(dashboard.telemetry, robot.opMode.telemetry)

    fun pre() {
        currentPacket = TelemetryPacket()
        drawRobot(currentPacket)
    }

    fun post() {
        dashboard.sendTelemetryPacket(currentPacket)
        telemetry.update()
    }

    fun drawRobot(packet: TelemetryPacket) {
        telemetry.addData("Pose", robot.drive.pose.toString())
        val canvas = packet.fieldOverlay()
        canvas.setStrokeWidth(robotStroke)
        canvas.setStroke(robotColor)
        val pose = robot.drive.pose
        canvas.strokeCircle(pose.toRR().x, pose.toRR().y, robotSize / 2)
        val lookVector = Vec2d(robotSize / 2, 0.0).rotate(pose.heading)
        val lookPoint = pose + lookVector
        canvas.strokeLine(pose.toRR().x, pose.toRR().y, lookPoint.toRR().x, lookPoint.toRR().y)
    }

    companion object {
        @JvmField
        var robotSize = 14.0

        @JvmField
        var robotColor = "#ffffff"

        @JvmField
        var robotStroke = 1
    }
}
