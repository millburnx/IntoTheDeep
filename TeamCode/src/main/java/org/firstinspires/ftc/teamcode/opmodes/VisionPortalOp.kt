package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.subsystems.Apriltag
import org.firstinspires.ftc.teamcode.common.subsystems.SampleDetector
import org.firstinspires.ftc.teamcode.common.subsystems.VisionPortal

@Config
@TeleOp(name = "Vision")
class VisionPortalOp : CommandOpMode() {
    lateinit var aprilTag: Apriltag
    lateinit var sampleDetector: SampleDetector
    lateinit var visionPortal: VisionPortal
    lateinit var tel: Telemetry

    override fun initialize() {
        aprilTag = Apriltag()
        sampleDetector = SampleDetector()
        visionPortal = VisionPortal(hardwareMap, "camera1", listOf(aprilTag.processor, sampleDetector))
        tel = FtcDashboard.getInstance().telemetry
        FtcDashboard.getInstance().startCameraStream(sampleDetector, 0.0)

        initCheck()
    }

    fun initCheck() {
        require(::aprilTag.isInitialized) { "AprilTag is not initialized" }
        require(::sampleDetector.isInitialized) { "sampleDetector is not initialized" }
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
        tel.update()
    }
}