package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.PIDCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonConfig

@Config
@TeleOp(name = "DriveTuner2", group = "Tuning")
class DrivePIDTuner2 : CommandOpMode() {
    val telem = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    val drive: Drive by lazy {
        Drive(hardwareMap, Telemetry(), FtcDashboard.getInstance())
    }

    val pidX = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidY = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidH = APIDController(AutonConfig.pidH_kP, AutonConfig.pidH_kI, AutonConfig.pidH_kD)

    override fun initialize() {
        pidX.reset()
        pidY.reset()
        pidH.reset()

        schedule(
            PIDCommand(
                drive,
                Vec2d(targetX, targetY),
                targetH,
                pidX,
                pidY,
                pidH,
            )
        )
    }

    override fun run() {
        super.run()
        telem.update()
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