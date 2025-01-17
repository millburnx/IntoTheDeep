package org.firstinspires.ftc.teamcode.common.processors

import android.graphics.Bitmap
import android.graphics.Canvas
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
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
class SampleDetector :
    VisionProcessor,
    CameraStreamSource {
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

    override fun init(
        width: Int,
        height: Int,
        calibration: CameraCalibration?,
    ) {
        FtcDashboard.getInstance().startCameraStream(this, 0.0)
    }

    val ycrcbMat = Mat()

    val redThres = Mat()
    val yellowThres = Mat()
    val blueThres = Mat()

    val channels: MutableList<Mat> = mutableListOf()
    val redContours: MutableList<MatOfPoint> = mutableListOf()
    val yellowContours: MutableList<MatOfPoint> = mutableListOf()
    val blueContours: MutableList<MatOfPoint> = mutableListOf()
    val redHierarchy = Mat()
    val yellowHierarchy = Mat()
    val blueHierarchy = Mat()

    val redSamples: AtomicReference<List<IDetection>> = AtomicReference(emptyList())
    val yellowSamples: AtomicReference<List<IDetection>> = AtomicReference(emptyList())
    val blueSamples: AtomicReference<List<IDetection>> = AtomicReference(emptyList())

    val masked = Mat()

    override fun processFrame(
        frame: Mat,
        captureTimeNanos: Long,
    ): Any? {
        channels.clear()
        redContours.clear()
        yellowContours.clear()
        blueContours.clear()

        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)
        Core.split(ycrcbMat, channels)

        Imgproc.threshold(
            channels[1],
            redThres,
            redLower,
            redUpper,
            Imgproc.THRESH_BINARY,
        )
        Imgproc.threshold(
            channels[2],
            yellowThres,
            yellowLower,
            yellowUpper,
            Imgproc.THRESH_BINARY_INV,
        )
        Imgproc.threshold(
            channels[2],
            blueThres,
            blueLower,
            blueUpper,
            Imgproc.THRESH_BINARY,
        )

        Imgproc.findContours(redThres, redContours, redHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.findContours(
            yellowThres,
            yellowContours,
            yellowHierarchy,
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_SIMPLE,
        )
        Imgproc.findContours(blueThres, blueContours, blueHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

        val redData = contoursToData(redContours)
        val yellowData = contoursToData(yellowContours)
        val blueData = contoursToData(blueContours)

        Imgproc.drawContours(masked, redData.map { it.contour }, -1, Scalar(255.0, 0.0, 255.0), 2)
        Imgproc.drawContours(masked, yellowData.map { it.contour }, -1, Scalar(255.0, 0.0, 255.0), 2)
        Imgproc.drawContours(masked, blueData.map { it.contour }, -1, Scalar(255.0, 0.0, 255.0), 2)

        redSamples.set(dataToSamples(redData, SampleColor.RED))
        yellowSamples.set(dataToSamples(yellowData, SampleColor.YELLOW))
        blueSamples.set(dataToSamples(blueData, SampleColor.BLUE))

        val contoursBitmap = Bitmap.createBitmap(masked.width(), masked.height(), Bitmap.Config.RGB_565)
        Utils.matToBitmap(masked, contoursBitmap)
        lastFrame.set(contoursBitmap)

        return null
    }

    fun contoursToData(contours: List<MatOfPoint>): List<DetectionData> {
        return contours
            .map { contour ->
                val rect = Imgproc.minAreaRect(MatOfPoint2f(*contour.toArray()))
                val area = rect.size.width * rect.size.height
                val ratio = min(rect.size.height / rect.size.width, rect.size.width / rect.size.height)
                if (area > areaMin && area < areaMax && ratio > ratioMin && ratio < ratioMax) {
                    return@map DetectionData(rect, area, ratio, contour)
                }
                return@map null
            }.filterNotNull()
    }

    fun dataToSamples(
        data: List<DetectionData>,
        color: SampleColor,
    ): List<SampleDetection> {
        for (detection in data) {
            val rect = detection.rect
            val area = rect.size.width * rect.size.height
            val ratio = min(rect.size.height / rect.size.width, rect.size.width / rect.size.height)
            Imgproc.putText(
                masked,
                String.format("%.2f | %.2f | %.2f", ratio, area, correctCVAngle(rect)),
                rect.center,
                Imgproc.FONT_HERSHEY_PLAIN,
                1.0,
                Scalar(0.0, 255.0, 0.0),
            )
        }

        return data.map { data ->
            val center = Vec2d(data.rect.center.x, data.rect.center.y)
            SampleDetection(
                center,
                correctCVAngle(data.rect),
                color,
                BoundingBox(
                    center,
                    Vec2d(data.rect.size.width, data.rect.size.height),
                ),
            )
        }
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
        userContext: Any?,
    ) {
    }

    val lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>) {
        continuation.dispatch { consumer -> consumer.accept(lastFrame.get()) }
    }
}

enum class SampleColor {
    RED,
    YELLOW,
    BLUE,
}

data class DetectionData(
    val rect: RotatedRect,
    val area: Double,
    val ratio: Double,
    val contour: MatOfPoint,
)

data class BoundingBox(
    val center: Vec2d,
    val size: Vec2d,
) {
    val area: Double
        get() = size.x * size.y
}

interface IDetection {
    val pos: Vec2d
    val angle: Double
    val boundingBox: BoundingBox
}

data class Detection(
    override val pos: Vec2d,
    override val angle: Double,
    override val boundingBox: BoundingBox,
) : IDetection

data class SampleDetection(
    override val pos: Vec2d,
    override val angle: Double,
    val color: SampleColor,
    override val boundingBox: BoundingBox,
) : IDetection
