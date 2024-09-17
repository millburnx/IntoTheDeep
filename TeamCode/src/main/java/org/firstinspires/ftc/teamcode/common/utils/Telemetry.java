package org.firstinspires.ftc.teamcode.common.utils;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.millburnx.utils.Vec2d;

import java.util.List;


public class Telemetry {
    Pose2d pos;
    MultipleTelemetry telemetry;

    public Telemetry() {
    }

    public static void drawRobot(Canvas canvas, Pose2d pose, String color) {
        canvas.setStroke(color);
        Pose2d transformed = toRR(pose);
        canvas.strokeCircle(transformed.getX(), transformed.getY(), 9);
        Vec2d lookVector = new Vec2d(9, 0).rotate(pose.getHeading()).toRR();
        Vec2d lookPoint = new Vec2d(transformed.getX(), transformed.getY()).plus(lookVector);
        canvas.strokeLine(transformed.getX(), transformed.getY(), lookPoint.getX(), lookPoint.getY());
    }

    public static Pose2d toRR(Pose2d pose) {
        return new Pose2d(pose.getY(), -pose.getX(), pose.getRotation());
    }

    //draw all the robots on the field and send to the dashboard
    public void drawField(List<PoseColor> poses, FtcDashboard dash) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#3F51B5");
//        drawRobot(fieldOverlay, pose, "#0000ff");
//
//        packet.put("x", pose.getX());
//        packet.put("y", pose.getY());
//        packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));

        for (int i = 0; i < poses.size(); i++) {
            Pose2d pose = poses.get(i).pose;
            drawRobot(fieldOverlay, pose, poses.get(i).color);
            packet.put(i + " | x", pose.getX());
            packet.put(i + " | y", pose.getY());
            packet.put(i + " | heading (deg)", Math.toDegrees(pose.getHeading()));
        }

        dash.sendTelemetryPacket(packet);
    }
}

