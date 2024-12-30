package org.firstinspires.ftc.teamcode.common.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.millburnx.dashboard.TelemetryPacket
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.Robot

@Config
class TelemetryManager(val robot: Robot) {
    val dashboard = FtcDashboard.getInstance()
    var currentPacket: TelemetryPacket = TelemetryPacket()

    val addData = currentPacket::put

    fun pre() {
        currentPacket = TelemetryPacket()
        drawRobot(currentPacket)
    }

    fun post() {
        dashboard.sendTelemetryPacket(currentPacket)
    }

    fun drawRobot(packet: TelemetryPacket) {
        packet.put("Pose", robot.drive.pose.toString())
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
        var robotColor = "#3F51B5"

        @JvmField
        var robotStroke = 1
    }
}