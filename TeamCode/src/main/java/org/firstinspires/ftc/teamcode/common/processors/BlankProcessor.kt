package org.firstinspires.ftc.teamcode.common.processors

import android.graphics.Bitmap
import android.graphics.Canvas
import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Mat
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