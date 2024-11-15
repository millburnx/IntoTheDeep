package org.firstinspires.ftc.teamcode.common.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.geometry.Pose2d
import com.millburnx.utils.Vec2d


class Telemetry(val telemetry: MultipleTelemetry? = null) {

    //draw all the robots on the field and send to the dashboard
    fun drawField(poses: List<TelemetryPose>, dash: FtcDashboard) {
        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()

        fieldOverlay.setStrokeWidth(1)
        fieldOverlay.setStroke("#3F51B5")

        for (i in poses.indices) {
            val robot = poses[i]
            drawRobot(fieldOverlay, robot.pose, robot.color, robot.size ?: 9.0)
            packet.put("$i | x", robot.pose.x)
            packet.put("$i | y", robot.pose.y)
            packet.put("$i | heading (deg)", Math.toDegrees(robot.pose.heading))
        }

        dash.sendTelemetryPacket(packet)
    }

    companion object {
        fun drawRobot(canvas: Canvas, pose: Pose2d, color: String?, size: Double = 9.0) {
            canvas.setStroke(color)
            val transformed: Pose2d = toRR(pose)
            canvas.strokeCircle(transformed.x, transformed.y, size)
            val lookVector = Vec2d(9, 0).rotate(pose.heading).toRR()
            val lookPoint = Vec2d(transformed.x, transformed.y).plus(lookVector)
            canvas.strokeLine(transformed.x, transformed.y, lookPoint.x, lookPoint.y)
        }

        fun toRR(pose: Pose2d): Pose2d {
            return Pose2d(pose.y, -pose.x, pose.rotation)
        }
    }
}

