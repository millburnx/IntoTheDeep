package org.firstinspires.ftc.teamcode.opmodes

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.geometry.Pose2d
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import java.io.File

@Config
object AutonConfig {
    @JvmField
    var multiF = 0.1

    @JvmField
    var multiH = 0.1

    @JvmField
    var pathName = "fancy"
}

@TeleOp(name = "Auton")
class Auton : CommandOpMode() {
    private var drive: DriveSubsystem? = null
    var tel: Telemetry? = null
    var pos: Pose2d = Pose2d()
    var dash: FtcDashboard? = null

    val path = {
        println("FILE PATH: ${Environment.getExternalStorageDirectory()} | ${Environment.getDataDirectory()}")
        var rootDir = Environment.getExternalStorageDirectory()
        val points = Vec2d.loadList(File("${rootDir}/Paths/${AutonConfig.pathName}.tsv"))
        points
    }()

    override fun initialize() {
        val drive = DriveSubsystem(hardwareMap)
        this.drive = drive
        tel = Telemetry()
        dash = FtcDashboard.getInstance()

        schedule(PurePursuitCommand(drive, path, dash!!))
    }

    override fun run() {
        drive!!.updatePos()
        super.run()
    }
}