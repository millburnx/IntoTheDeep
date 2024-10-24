package org.firstinspires.ftc.teamcode.opmodes

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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
}

@TeleOp(name = "Auton")
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
            g
            e.printStackTrace()
            println("PATH (`$path`) NOT FOUND")
            listOf<Vec2d>()
        }
    }

    override fun initialize() {
        val drive = Drive(hardwareMap, tel, dash)
        this.drive = drive

        schedule(PurePursuitCommand(drive, path, dash))
    }

    override fun run() {
        super.run()
    }
}