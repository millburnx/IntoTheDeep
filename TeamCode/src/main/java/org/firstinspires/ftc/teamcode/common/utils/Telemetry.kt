package org.firstinspires.ftc.teamcode.common.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.geometry.Pose2d
import com.millburnx.utils.Vec2d


class Telemetry {
    var pos: Pose2d? = null
    var telemetry: MultipleTelemetry? = null

    //draw all the robots on the field and send to the dashboard
    fun drawField(poses: MutableList<PoseColor>, dash: FtcDashboard) {
        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()

        fieldOverlay.setStrokeWidth(1)
        fieldOverlay.setStroke("#3F51B5")

        //        drawRobot(fieldOverlay, pose, "#0000ff");
//
//        packet.put("x", pose.getX());
//        packet.put("y", pose.getY());
//        packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));
        for (i in poses.indices) {
            val pose = poses.get(i)!!.pose
            drawRobot(fieldOverlay, pose, poses.get(i)!!.color)
            packet.put(i.toString() + " | x", pose.getX())
            packet.put(i.toString() + " | y", pose.getY())
            packet.put(i.toString() + " | heading (deg)", Math.toDegrees(pose.getHeading()))
        }

        dash.sendTelemetryPacket(packet)
    }

    companion object {
        fun drawRobot(canvas: Canvas, pose: Pose2d, color: String?) {
            canvas.setStroke(color)
            val transformed: Pose2d = toRR(pose)
            canvas.strokeCircle(transformed.x, transformed.y, 9.0)
            val lookVector = Vec2d(9, 0).rotate(pose.heading).toRR()
            val lookPoint = Vec2d(transformed.x, transformed.y).plus(lookVector)
            canvas.strokeLine(transformed.x, transformed.y, lookPoint.x, lookPoint.y)
        }

        fun toRR(pose: Pose2d): Pose2d {
            return Pose2d(pose.y, -pose.x, pose.rotation)
        }
    }
}

