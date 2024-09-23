package org.firstinspires.ftc.teamcode.common.subsystems

import android.graphics.Bitmap
import android.graphics.Canvas
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import java.util.ArrayList
import java.util.concurrent.atomic.AtomicReference

class SampleDetection : SubsystemBase() {

}

@Config
object SampleDetectorConfig {
    @JvmField
    var color = 0
}

class SampleDetector: VisionProcessor, CameraStreamSource {
    val lastFrame: AtomicReference<Bitmap> = AtomicReference(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565))

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(
        frame: Mat,
        captureTimeNanos: Long
    ): Any? {
        val ycrcb = Mat()
        Imgproc.cvtColor(frame, ycrcb, Imgproc.COLOR_RGB2YCrCb)

        val crChannel = Mat()
        val cbChannel = Mat()
        Core.extractChannel(ycrcb, crChannel, 1)
        Core.extractChannel(ycrcb, cbChannel, 2)

        val rThreshold = Mat()
        val yThreshold = Mat()
        val bThreshold = Mat()
        Imgproc.threshold(crChannel, rThreshold, 190.0, 255.0, Imgproc.THRESH_BINARY) // green
        Imgproc.threshold(cbChannel, yThreshold, 57.0, 255.0, Imgproc.THRESH_BINARY)  // white
        Imgproc.threshold(cbChannel, bThreshold, 150.0, 255.0, Imgproc.THRESH_BINARY) // red

        val bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
        if (SampleDetectorConfig.color == 0) {
            Utils.matToBitmap(rThreshold, bitmap)
            lastFrame.set(bitmap)
        } else if (SampleDetectorConfig.color == 1) {
            Utils.matToBitmap(yThreshold, bitmap)
            lastFrame.set(bitmap)
        } else if (SampleDetectorConfig.color == 2) {
            Utils.matToBitmap(bThreshold, bitmap)
            lastFrame.set(bitmap)
        }
        return frame
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        TODO("Not yet implemented")
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap?>?>?) {
        TODO("Not yet implemented")
    }

}