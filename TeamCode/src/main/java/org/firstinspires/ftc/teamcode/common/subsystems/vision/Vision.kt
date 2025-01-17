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

    open val processors =
        listOf<VisionProcessor>(
            BlankProcessor(),
        )

    val visionPortal: VisionPortal =
        VisionPortal
            .Builder()
            .setCamera(robot.hardware["Webcam 1"] as WebcamName)
            .addProcessors(*processors.toTypedArray())
            .setCameraResolution(Size(cameraSize.x.toInt(), cameraSize.y.toInt()))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build()

    companion object {
        @JvmField
        var exposureTime: Long = 10L

        @JvmField
        var gain: Int = 255
    }

    override fun periodic() {
        if (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) return
        val exposureController = visionPortal.getCameraControl(ExposureControl::class.java)
        val gainController = visionPortal.getCameraControl(GainControl::class.java)
        exposureController.setExposure(exposureTime, TimeUnit.MILLISECONDS)
        gainController.gain = gain
    }
}
