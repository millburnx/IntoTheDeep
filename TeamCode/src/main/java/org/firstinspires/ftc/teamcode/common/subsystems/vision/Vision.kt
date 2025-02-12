package org.firstinspires.ftc.teamcode.common.subsystems.vision

import android.util.Size
import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.processors.BlankProcessor
import org.firstinspires.ftc.teamcode.common.processors.SampleDetector
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import java.util.concurrent.TimeUnit

@Config
open class Vision(
    robot: Robot,
) : Subsystem() {
    val cameraSize = Vec2d(640, 480)

    open val processors1 =
        listOf<VisionProcessor>(
            BlankProcessor(),
        )

    val multiPortal = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL)

    val camera1: VisionPortal by lazy {
        VisionPortal
            .Builder()
            .setCamera(robot.hardware["Webcam 1"] as WebcamName)
            .addProcessors(*processors1.toTypedArray())
            .setCameraResolution(Size(cameraSize.x.toInt(), cameraSize.y.toInt()))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .setLiveViewContainerId(multiPortal[0])
            .setAutoStopLiveView(true)
            .build()
    }

    override fun init() {
        camera1
    }

    companion object {
        @JvmField
        var exposureTime1: Long = 7L

        @JvmField
        var gain1: Int = 255
    }

    fun update() {
        println("vision periodic")
        println(camera1)
        if (camera1.cameraState == VisionPortal.CameraState.STREAMING) {
            println("vision setting")
            val exposureController = camera1.getCameraControl(ExposureControl::class.java)
            val gainController = camera1.getCameraControl(GainControl::class.java)
            exposureController.mode = ExposureControl.Mode.Manual
            exposureController.setExposure(exposureTime1, TimeUnit.MILLISECONDS)
            gainController.gain = gain1
        }
    }
}

class SampleVision(
    robot: Robot,
) : Vision(robot) {
    val sampleDetector1 = SampleDetector()
//    val sampleDetector2 = SampleDetector()

    override val processors1 =
        listOf<VisionProcessor>(
            sampleDetector1,
//            BlankProcessor(),
        )
}
