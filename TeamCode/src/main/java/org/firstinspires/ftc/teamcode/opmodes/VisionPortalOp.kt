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
    val aprilTag: Apriltag = Apriltag()
    val samplePipeline: SamplePipeline = SamplePipeline()
    val visionPortal: VisionPortal = VisionPortal(hardwareMap, "camera1", listOf(aprilTag.processor, samplePipeline))
    val tel: Telemetry = FtcDashboard.getInstance().telemetry

    override fun initialize() {
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)
    }

    override fun run() {
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