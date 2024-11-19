package org.firstinspires.ftc.teamcode.opmodes.misc

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.teamcode.common.subsystems.vision.FasterSampleDetection
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleColor
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SamplePipeline
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal
import java.util.concurrent.TimeUnit

@Config
@TeleOp(name = "Camera Debugger", group = "Misc")
class CameraDebugger : CommandOpMode() {
    val samplePipeline: SamplePipeline by lazy { SamplePipeline() }
    val fasterPipeline: FasterSampleDetection by lazy { FasterSampleDetection(telemetry) }
    val visionPortal: VisionPortal by lazy {
        VisionPortal(
            hardwareMap,
            "camera1",
            listOf(samplePipeline, fasterPipeline)
        )
    }
    val tel: Telemetry = FtcDashboard.getInstance().telemetry

    override fun initialize() {
        visionPortal
    }

    var lastExposure = exposureTime
    var lastGain = gain

    var lastSample = enableSample
    var lastFaster = !enableFaster

    override fun run() {
        super.run()
        visionPortal.visionPortal.setProcessorEnabled(samplePipeline, enableSample)
        if (enableSample && !lastSample) {
            FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)
        } else if (!enableSample && lastSample || enableFaster && !lastFaster) {
            FtcDashboard.getInstance().startCameraStream(fasterPipeline, 0.0)
        }

        lastSample = enableSample
        lastFaster = enableFaster

        val exposure = visionPortal.visionPortal.getCameraControl(ExposureControl::class.java)
        val gain = visionPortal.visionPortal.getCameraControl(GainControl::class.java)
        val whiteBalance = visionPortal.visionPortal.getCameraControl(WhiteBalanceControl::class.java)
        val focus = visionPortal.visionPortal.getCameraControl(FocusControl::class.java)
        val panTilt = visionPortal.visionPortal.getCameraControl(PtzControl::class.java)

        exposure.mode = ExposureControl.Mode.Manual
        if (lastExposure != exposureTime) {
            tel.addData("Exposure Set", exposure.setExposure(exposureTime, TimeUnit.MILLISECONDS))
            lastExposure = exposureTime
        }
        if (lastGain != Companion.gain) {
            tel.addData("Gain Set", gain.setGain(Companion.gain))
            lastGain = Companion.gain
        }

        tel.addData("Exposure Support", exposure.isExposureSupported)
        for (exposureMode in ExposureControl.Mode.entries) {
            tel.addData("Exposure $exposureMode", exposure.isModeSupported(exposureMode))
        }

        tel.addData("Focus Support", focus.isFocusLengthSupported)
        for (focusMode in FocusControl.Mode.entries) {
            tel.addData("Focus $focusMode", focus.isModeSupported(focusMode))
        }

        if (enableSample) {
            val samples = samplePipeline.detections.get()

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
        var enableSample: Boolean = false

        @JvmField
        var enableFaster: Boolean = true

        @JvmField
        var exposureTime: Long = 10

        @JvmField
        var gain: Int = 255
    }
}