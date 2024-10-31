package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import java.io.File
import java.lang.Error

@Config
object AutonConfig {
    @JvmField
    var multiF = 0.75

    @JvmField
    var multiH = 0.75

    @JvmField
    var pathName = "intothedeep"

    @JvmField
    var useEndingHeading = false

    @JvmField
    var endingHeading = 0.0

    @JvmField
    var pidT_kP = 0.0

    @JvmField
    var pidT_kI = 0.0

    @JvmField
    var pidT_kD = 0.0

    @JvmField
    var pidH_kP = 0.0

    @JvmField
    var pidH_kI = 0.0

    @JvmField
    var pidH_kD = 0.0
}

@Autonomous(name = "Auton")
class Auton : CommandOpMode() {
    var drive: Drive? = null
    val tel: Telemetry = Telemetry()
    val dash: FtcDashboard = FtcDashboard.getInstance()

    val path = run {
        println("FILE DIR: ${Environment.getExternalStorageDirectory()} | ${Environment.getDataDirectory()}")
        var rootDir = Environment.getExternalStorageDirectory()
        val path = "${rootDir}/Paths/${AutonConfig.pathName}.tsv"
        try {
            val points = Vec2d.loadList(File(path))
            points
        } catch (e: Error) {
            e.printStackTrace()
            println("PATH (`$path`) NOT FOUND")
            listOf<Vec2d>()
        }
    }

    override fun initialize() {
        val drive = Drive(hardwareMap, tel, dash)
        this.drive = drive

        val pidX = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
        val pidY = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
        val pidH = PIDController(AutonConfig.pidH_kP, AutonConfig.pidH_kI, AutonConfig.pidH_kD)
        schedule(
            PurePursuitCommand(
                drive,
                path,
                if (AutonConfig.useEndingHeading) AutonConfig.endingHeading else null,
                dash,
                pidX,
                pidY,
                pidH
            )
        )
    }

    override fun run() {
        super.run()
    }
}