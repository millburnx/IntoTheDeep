package org.firstinspires.ftc.teamcode.opmodes.misc

import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.utils.Telemetry

@TeleOp(name = "Encoder", group = "Misc")
class EncoderTest() : CommandOpMode() {
    val tel: Telemetry by lazy {
        Telemetry()
    }
    val drive: Drive by lazy {
        Drive(hardwareMap, tel, FtcDashboard.getInstance())
    }

    override fun initialize() {
        drive.leftOdom.reset()
        drive.rightOdom.reset()
        drive.rightOdom.reset()
    }

    override fun run() {
        FtcDashboard.getInstance().telemetry.addData("left", drive.leftOdom.position)
        FtcDashboard.getInstance().telemetry.addData("center", drive.centerOdom.position)
        FtcDashboard.getInstance().telemetry.addData("right", drive.rightOdom.position)
        FtcDashboard.getInstance().telemetry.update()
    }
}