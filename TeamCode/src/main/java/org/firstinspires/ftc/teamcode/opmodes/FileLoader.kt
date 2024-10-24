package org.firstinspires.ftc.teamcode.opmodes

import android.os.Environment
import com.arcrobotics.ftclib.command.CommandOpMode
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.io.File

@TeleOp(name = "FileLoader")
class FileLoader : CommandOpMode() {
    override fun initialize() {
        try {
            println("FILE PATH: ${Environment.getExternalStorageDirectory()} | ${Environment.getDataDirectory()}")
            var rootDir = Environment.getExternalStorageDirectory()
            val points = Vec2d.loadList(File("${rootDir}/Paths/intothedeep.tsv"))
            val path = Utils.pathToBeziers(points)
            println(path.joinToString("\n"))
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    override fun run() {
        super.run()
    }
}