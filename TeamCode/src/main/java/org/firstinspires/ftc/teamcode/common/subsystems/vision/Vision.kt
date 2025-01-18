package org.firstinspires.ftc.teamcode.common.subsystems.vision

import android.util.Size
import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.processors.BlankProcessor
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

    open val processors2 =
        listOf<VisionProcessor>(
            BlankProcessor(),
        )

    val multiPortal = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL)

    val camera1: VisionPortal =
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

    val camera2: VisionPortal =
        VisionPortal
            .Builder()
            .setCamera(robot.hardware["Webcam 2"] as WebcamName)
            .addProcessors(*processors2.toTypedArray())
            .setCameraResolution(Size(cameraSize.x.toInt(), cameraSize.y.toInt()))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .setLiveViewContainerId(multiPortal[1])
            .setAutoStopLiveView(true)
            .build()

    companion object {
        @JvmField
        var exposureTime1: Long = 10L

        @JvmField
        var gain1: Int = 255

        @JvmField
        var exposureTime2: Long = 10L

        @JvmField
        var gain2: Int = 255
    }

    override fun periodic() {
        println("vision periodic")
        println(camera1)
        println(camera2)
        if (camera1.cameraState == VisionPortal.CameraState.STREAMING) {
            println("vision setting")
            val exposureController = camera1.getCameraControl(ExposureControl::class.java)
            val gainController = camera1.getCameraControl(GainControl::class.java)
            exposureController.setExposure(exposureTime1, TimeUnit.MILLISECONDS)
            gainController.gain = gain1
        }
        if (camera2.cameraState == VisionPortal.CameraState.STREAMING) {
            println("vision setting")
            val exposureController = camera2.getCameraControl(ExposureControl::class.java)
            val gainController = camera2.getCameraControl(GainControl::class.java)
            exposureController.setExposure(exposureTime2, TimeUnit.MILLISECONDS)
            gainController.gain = gain2
        }
    }
}

class SampleVision(
    robot: Robot,
) : Vision(robot) {
//    val sampleDetector1 = SampleDetector()
//    val sampleDetector2 = SampleDetector()

//    override val processors1 =
//        listOf<VisionProcessor>(
// //            sampleDetector1,
//            BlankProcessor(),
//        )
//
//    override val processors2 =
//        listOf(
// //            sampleDetector2,
//            BlankProcessor(),
//        )
}
