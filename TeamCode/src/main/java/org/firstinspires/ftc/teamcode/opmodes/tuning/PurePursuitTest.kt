package org.firstinspires.ftc.teamcode.opmodes.tuning

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Path
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.vision.FasterSampleDetection
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal
import org.firstinspires.ftc.teamcode.common.utils.APIDController
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonConfig
import java.io.File

@Autonomous(name = "Pure Pursuit")
class PurePursuitTest : CommandOpMode() {
    companion object {
        @JvmField
        var startX = 0.0

        @JvmField
        var startY = 0.0

        @JvmField
        var startH = 0.0

        val pathMap: MutableMap<String, Path> = mutableMapOf()

        @JvmField
        var pathName = "test"
    }

    val drive: Drive by lazy {
        Drive(
            hardwareMap,
            tel,
            dash,
            Vec2d(startX, startY),
            startH,
            zeroBreak = true
        )
    }
    val arm: Arm by lazy {
        Arm(hardwareMap, telemetry, lift.lift::getCurrentPosition)
    }
    val lift: Lift by lazy {
        Lift(hardwareMap)
    }

    val tel: Telemetry = Telemetry()
    val dash: FtcDashboard = FtcDashboard.getInstance()
    val paths = mutableListOf<Path>()

    val rootDir = Environment.getExternalStorageDirectory()

    fun loadSegment(index: Int): Path {
        return loadPath("${AutonConfig.pathName}${index}")
    }

    fun loadPath(name: String): Path {
        return pathMap.getOrPut(name) {
            val path = "${rootDir}/Paths/${name}.tsv"
            val segment: Path = try {
                val loaded = Vec2d.loadList(File(path))
                println(loaded)
                loaded
            } catch (e: Error) {
                e.printStackTrace()
                println("PATH (`$path`) NOT FOUND")
                Path(listOf())
            }
            return segment
        }
    }

    val pidX = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidY = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidH = APIDController(AutonConfig.pidH_kP, AutonConfig.pidH_kI, AutonConfig.pidH_kD)

    val samplePipeline: FasterSampleDetection by lazy { FasterSampleDetection(telemetry) }
    val visionPortal: VisionPortal by lazy {
        VisionPortal(
            hardwareMap,
            "camera1",
            listOf(samplePipeline)
        )
    }

    fun purePursuitSegment(segment: Path) = PurePursuitCommand(
        drive,
        segment.points,
        segment.endHeading,
        dash,
        pidX,
        pidY,
        pidH,
        AutonConfig.minRange..AutonConfig.maxRange
    ).withTimeout(AutonConfig.timeout)

    override fun initialize() {
        //start at (-60, 12)
        lift.armAngle = arm::angle
        visionPortal
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)

        val commands: MutableList<Command> = mutableListOf()
        commands.add(purePursuitSegment(loadPath(pathName)))
        schedule(
            SequentialCommandGroup(*commands.toTypedArray())
        )
    }

    override fun run() {
        super.run()
    }
}