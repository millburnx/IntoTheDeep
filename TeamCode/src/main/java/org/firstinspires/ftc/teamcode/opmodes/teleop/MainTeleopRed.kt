package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Config
@TeleOp(name = "Main Teleop Red")
class MainTeleopRed : MainTeleopBlue() {
    override fun initialize() {
        robot.isRed = true
        FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector, 0.0)
        triggers
    }
}
