package org.firstinspires.ftc.teamcode.opmodes.misc

import android.os.Environment
import com.arcrobotics.ftclib.command.CommandOpMode
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.io.File

@Disabled
@TeleOp(name = "FileLoader", group = "Misc")
class FileLoader : CommandOpMode() {
    override fun initialize() {
        try {
            println("FILE PATH: ${Environment.getExternalStorageDirectory()} | ${Environment.getDataDirectory()}")
            var rootDir = Environment.getExternalStorageDirectory()
            val points = Vec2d.loadList(File("${rootDir}/Paths/intothedeep.tsv")).points
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