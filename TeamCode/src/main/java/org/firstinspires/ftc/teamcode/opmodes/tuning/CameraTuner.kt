package org.firstinspires.ftc.teamcode.opmodes.tuning

import android.graphics.Bitmap
import android.graphics.Canvas
import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Mat
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicReference

class BlankProcessor : VisionProcessor, CameraStreamSource {

    val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun init(
        width: Int,
        height: Int,
        calibration: CameraCalibration
    ) {
        FtcDashboard.getInstance().startCameraStream(this, 0.0)
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        val contoursBitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
        Utils.matToBitmap(frame, contoursBitmap)
        lastFrame.set(contoursBitmap)
        return null
    }

    override fun onDrawFrame(
        p0: Canvas,
        p1: Int,
        p2: Int,
        p3: Float,
        p4: Float,
        p5: Any?
    ) {
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>) {
        continuation.dispatch { it.accept(lastFrame.get()) }
    }
}

@Config
class Vision(robot: Robot) : Subsystem() {
    val cameraSize = Vec2d(640, 480)

    val processors = listOf<VisionProcessor>(
        BlankProcessor()
    )

    val visionPortal = VisionPortal.Builder()
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

class CameraRobot(opMode: OpMode) : Robot(opMode) {
    val camera = Vision(this)
    override val additionalSubsystems = listOf(camera)

    override fun init() {
        imu.resetYaw()
        super.init()
    }
}

@TeleOp(name = "Camera Tuner")
class CameraTuner : OpMode() {
    override val robot by lazy { CameraRobot(this) }

    override fun exec() {
    }
}