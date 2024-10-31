package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonConfig

@Config
@TeleOp(name = "DriveTuner", group = "Tuning")
class DrivePIDTuner : CommandOpMode() {
    val telem = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    val drive: Drive by lazy {
        Drive(hardwareMap, Telemetry(), FtcDashboard.getInstance())
    }

    val pidX = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidY = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidH = PIDController(AutonConfig.pidH_kP, AutonConfig.pidH_kI, AutonConfig.pidH_kD)

    override fun initialize() {
        pidX.reset()
        pidY.reset()
        pidH.reset()
    }

    override fun run() {
        val pose = drive.pose
        pidX.setPID(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
        pidY.setPID(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
        pidH.setPID(AutonConfig.pidH_kP, AutonConfig.pidH_kI, AutonConfig.pidH_kD)
        val powerX = pidX.calculate(pose.x, targetX)
        val powerY = pidX.calculate(pose.y, targetX)
        val powerH = pidX.calculate(Math.toDegrees(pose.heading), targetX)

        telem.addData("powerX", powerX)
        telem.addData("powerY", powerY)
        telem.addData("powerH", powerH)
        telem.addData("poseX", pose.x)
        telem.addData("poseY", pose.y)
        telem.addData("poseH", Math.toDegrees(pose.heading))

        drive.fieldCentric(powerX, powerY, powerH, pose.heading)
    }

    companion object {
        @JvmField
        var targetX: Double = 0.0

        @JvmField
        var targetY: Double = 0.0

        @JvmField
        var targetH: Double = 0.0
    }
}