package org.firstinspires.ftc.teamcode.opmodes.misc

import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.subsystems.vision.ClipPipeline
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal

//@Disabled
@TeleOp(name = "Vision Clip", group = "Misc")
class VisionPortalOpClip : CommandOpMode() {
    lateinit var clipPipeline: ClipPipeline
    lateinit var visionPortal: VisionPortal
    lateinit var tel: Telemetry

    override fun initialize() {
        clipPipeline = ClipPipeline()
        visionPortal = VisionPortal(hardwareMap, "camera1", listOf(clipPipeline))
        tel = FtcDashboard.getInstance().telemetry
        FtcDashboard.getInstance().startCameraStream(clipPipeline, 0.0)

        initCheck()
    }

    fun initCheck() {
        require(::clipPipeline.isInitialized) { "Clip Pipeline is not initialized" }
        require(::visionPortal.isInitialized) { "Vision Portal is not initialized" }
        require(::tel.isInitialized) { "Telemetry is not initialized" }
    }

    override fun run() {
        initCheck()
        super.run()

        val clips = clipPipeline.detections.get()
        tel.addData("Clips", clips.size)
        clips.forEachIndexed { index, clip ->
            tel.addData("Clip $index", clip.toString())
        }
        tel.update()
    }
}