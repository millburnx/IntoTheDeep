package org.firstinspires.ftc.teamcode.common.subsystems.vision

import android.graphics.Bitmap
import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Mat
import java.util.concurrent.atomic.AtomicReference

class ClipPipeline : VisionProcessor, CameraStreamSource {
    val lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(
        frame: Mat,
        captureTimeNanos: Long
    ): Any? {
        val bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
        Utils.matToBitmap(frame, bitmap)
        lastFrame.set(bitmap)

        return frame
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        // todo
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>) {
        continuation.dispatch { consumer -> consumer.accept(lastFrame.get()) }
    }
}