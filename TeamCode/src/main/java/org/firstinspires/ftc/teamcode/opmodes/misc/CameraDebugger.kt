package org.firstinspires.ftc.teamcode.opmodes.misc

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleColor
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SamplePipeline
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal

@Config
@TeleOp(name = "Camera Debugger", group = "Misc")
class CameraDebugger : CommandOpMode() {
    val samplePipeline: SamplePipeline by lazy { SamplePipeline() }
    val visionPortal: VisionPortal by lazy { VisionPortal(hardwareMap, "camera1", listOf(samplePipeline)) }
    val tel: Telemetry = FtcDashboard.getInstance().telemetry

    override fun initialize() {
        visionPortal
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)
    }

    override fun run() {
        super.run()
        visionPortal.visionPortal.setProcessorEnabled(samplePipeline, enableSpecimens)

        val exposure = visionPortal.visionPortal.getCameraControl(ExposureControl::class.java)
        val whiteBalance = visionPortal.visionPortal.getCameraControl(WhiteBalanceControl::class.java)
        val focus = visionPortal.visionPortal.getCameraControl(FocusControl::class.java)
        val panTilt = visionPortal.visionPortal.getCameraControl(PtzControl::class.java)

        tel.clear()

        tel.addData("Exposure Support", exposure.isExposureSupported)
        for (exposureMode in ExposureControl.Mode.entries) {
            tel.addData("Exposure $exposureMode", exposure.isModeSupported(exposureMode))
        }

        tel.addData("Focus Support", focus.isFocusLengthSupported)
        for (focusMode in FocusControl.Mode.entries) {
            tel.addData("Focus $focusMode", focus.isModeSupported(focusMode))
        }

        if (enableSpecimens) {
            val samples = samplePipeline.detections.get()

//            samples.forEachIndexed { i, sample ->
//                tel.addData("sample $i", sample.toString())
//            }
            tel.addData("TOTAL SAMPLES: ", "${samples.size}")
            tel.addData("RED SAMPLES: ", "{${samples.filter { sample -> sample.color == SampleColor.RED }.size}}")
            tel.addData("YELLOW SAMPLES: ", "{${samples.filter { sample -> sample.color == SampleColor.YELLOW }.size}}")
            tel.addData("BLUE SAMPLES: ", "{${samples.filter { sample -> sample.color == SampleColor.BLUE }.size}}")
        }
        tel.addData("FPS", visionPortal.visionPortal.fps)
        tel.update()
    }

    companion object {
        @JvmField
        var enableSpecimens: Boolean = false
    }
}