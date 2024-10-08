package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.subsystems.Apriltag
import org.firstinspires.ftc.teamcode.common.subsystems.SampleColor
import org.firstinspires.ftc.teamcode.common.subsystems.SamplePipeline
import org.firstinspires.ftc.teamcode.common.subsystems.VisionPortal

@Config
@TeleOp(name = "Vision")
class VisionPortalOp : CommandOpMode() {
    lateinit var aprilTag: Apriltag
    lateinit var samplePipeline: SamplePipeline
    lateinit var visionPortal: VisionPortal
    lateinit var tel: Telemetry

    override fun initialize() {
        aprilTag = Apriltag()
        samplePipeline = SamplePipeline()
        visionPortal = VisionPortal(hardwareMap, "camera1", listOf(aprilTag.processor, samplePipeline))
        tel = FtcDashboard.getInstance().telemetry
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)

        initCheck()
    }

    fun initCheck() {
        require(::aprilTag.isInitialized) { "AprilTag is not initialized" }
        require(::samplePipeline.isInitialized) { "sampleDetector is not initialized" }
        require(::visionPortal.isInitialized) { "Vision Portal is not initialized" }
        require(::tel.isInitialized) { "Telemetry is not initialized" }
    }

    override fun run() {
        initCheck()
        super.run()

        val tags = aprilTag.getTags()

        for (tag in tags) {
            if (tag.ftcPose == null) return
            val poseData = "${tag.ftcPose.x}, ${tag.ftcPose.y}, ${tag.ftcPose.z}"
            tel.addData(tag.id.toString(), poseData)
        }

        val samples = samplePipeline.detections.get()

        samples.forEachIndexed { i, sample ->
            tel.addData("sample $i", sample.toString())
        }
        tel.addData("TOTAL SAMPLES: ", "${samples.size}")
        tel.addData("RED SAMPLES: ", "{${samples.filter { sample -> sample.color == SampleColor.RED }.size}}")
        tel.addData("YELLOW SAMPLES: ", "{${samples.filter { sample -> sample.color == SampleColor.YELLOW }.size}}")
        tel.addData("BLUE SAMPLES: ", "{${samples.filter { sample -> sample.color == SampleColor.BLUE }.size}}")

        tel.update()
    }
}