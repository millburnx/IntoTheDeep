package org.firstinspires.ftc.teamcode.common.subsystems.vision

import android.graphics.Bitmap
import android.graphics.Canvas
import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.opmodes.misc.CameraDebugger
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.RotatedRect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference
import kotlin.collections.mutableListOf
import kotlin.math.min

@Config
class FasterSampleDetection(val telemetry: Telemetry?) : VisionProcessor, CameraStreamSource {
    companion object {
        @JvmField
        var redLower: Double = 199.0

        @JvmField
        var redUpper: Double = 255.0

        @JvmField
        var yellowLower: Double = 90.0

        @JvmField
        var yellowUpper: Double = 255.0

        @JvmField
        var blueLower: Double = 150.0

        @JvmField
        var blueUpper: Double = 255.0

        @JvmField
        var areaMin: Double = 1_000.0

        @JvmField
        var areaMax: Double = 15_000.0

        @JvmField
        var ratioMin: Double = 0.375

        @JvmField
        var ratioMax: Double = 0.9
    }

    val detections: AtomicReference<List<IDetection>> = AtomicReference(emptyList())

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
    }

    val ycrcbMat = Mat()

    val redThres = Mat()
    val yellowThres = Mat()
    val blueThres = Mat()

    val merged = Mat()
    val hierarchy = Mat()

    val channels: MutableList<Mat> = mutableListOf<Mat>()
    val contours: MutableList<MatOfPoint> = mutableListOf<MatOfPoint>()

    val masked = Mat()

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        if (!CameraDebugger.enableFaster) {
            val contoursBitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
            Utils.matToBitmap(frame, contoursBitmap)
            lastFrame.set(contoursBitmap)
            return null
        }

        channels.clear()
        contours.clear()

        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)
        Core.split(ycrcbMat, channels)

        Imgproc.threshold(
            channels[1],
            redThres,
            redLower,
            redUpper,
            Imgproc.THRESH_BINARY
        )
        Imgproc.threshold(
            channels[2],
            yellowThres,
            yellowLower,
            yellowUpper,
            Imgproc.THRESH_BINARY_INV
        )
        Imgproc.threshold(
            channels[2],
            blueThres,
            blueLower,
            blueUpper,
            Imgproc.THRESH_BINARY
        )

//        Imgproc.morphologyEx(redThres, redThres, Imgproc.MORPH_CLOSE, new Mat());
//        Imgproc.morphologyEx(yellowThres, yellowThres, Imgproc.MORPH_CLOSE, new Mat());
//        Imgproc.morphologyEx(blueThres, blueThres, Imgproc.MORPH_CLOSE, new Mat());

        Core.bitwise_or(redThres, yellowThres, merged)
        Core.bitwise_or(merged, blueThres, merged)

        Imgproc.findContours(merged, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

        val finalData: List<Pair<MatOfPoint, RotatedRect>> = contours.map { contour ->
            val rect = Imgproc.minAreaRect(MatOfPoint2f(*contour.toArray()))
            val area = rect.size.width * rect.size.height
            val ratio = min(rect.size.height / rect.size.width, rect.size.width / rect.size.height)
            if (area > areaMin && area < areaMax && ratio > ratioMin && ratio < ratioMax) {
                return@map Pair(contour, rect)
            }
            return@map null
        }.filterNotNull()

        detections.set(finalData.map { data ->
            val center = Vec2d(data.second.center.x, data.second.center.y)
            Detection(
                center,
                correctCVAngle(data.second),
                BoundingBox(
                    center,
                    Vec2d(data.second.size.width, data.second.size.height)
                )
            )
        })

        masked.setTo(Scalar.all(0.0))
        Core.bitwise_and(frame, frame, masked, merged)
        Imgproc.drawContours(masked, finalData.map { it.first }, -1, Scalar(255.0, 0.0, 255.0), 2)
        for (detection in finalData) {
            val rect = detection.second
            val area = rect.size.width * rect.size.height
            val ratio = min(rect.size.height / rect.size.width, rect.size.width / rect.size.height)
            Imgproc.putText(
                masked,
                String.format("%.2f | %.2f | %.2f", ratio, area, correctCVAngle(rect)),
                rect.center,
                Imgproc.FONT_HERSHEY_PLAIN,
                1.0,
                Scalar(0.0, 255.0, 0.0)
            )
        }

        val contoursBitmap = Bitmap.createBitmap(masked.width(), masked.height(), Bitmap.Config.RGB_565)
        Utils.matToBitmap(masked, contoursBitmap)
        lastFrame.set(contoursBitmap)

        return null
    }

    fun correctCVAngle(rect: RotatedRect): Double {
        var angle = -rect.angle
        if (rect.size.width < rect.size.height) {
            angle += 90.0
        }
        return angle
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
    }

    val lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>) {
        continuation.dispatch { consumer -> consumer.accept(lastFrame.get()) }
    }
}

