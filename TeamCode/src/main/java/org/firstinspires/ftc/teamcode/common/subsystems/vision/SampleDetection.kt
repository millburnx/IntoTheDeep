package org.firstinspires.ftc.teamcode.common.subsystems.vision

import android.graphics.Bitmap
import android.graphics.Canvas
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.millburnx.util.Vec2d
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
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.round

class SampleDetector : SubsystemBase() {

}

@Config
object SampleDetectorConfig {
    @JvmField
    var color = 0
}

class SamplePipeline : VisionProcessor, CameraStreamSource {
    val lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    val detections: AtomicReference<List<SampleDetection>> = AtomicReference(listOf<SampleDetection>())

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
        Imgproc.threshold(crChannel, rThreshold, 190.0, 255.0, Imgproc.THRESH_BINARY)
        Imgproc.threshold(cbChannel, yThreshold, 57.0, 255.0, Imgproc.THRESH_BINARY_INV)
        Imgproc.threshold(cbChannel, bThreshold, 150.0, 255.0, Imgproc.THRESH_BINARY)

        val rContours = filterContours(getContours(rThreshold))
        val yContours = filterContours(getContours(yThreshold))
        val bContours = filterContours(getContours(bThreshold))

        val rDetections = contoursToDetections(rContours, SampleColor.RED)
        val yDetections = contoursToDetections(yContours, SampleColor.YELLOW)
        val bDetections = contoursToDetections(bContours, SampleColor.BLUE)

        val detections = rDetections + yDetections + bDetections
        this.detections.set(detections)

        val contoursImg = frame.clone()
        drawContours(
            contoursImg,
            rContours,
            Scalar(255.0, 0.0, 0.0),
            Scalar(0.0, 255.0, 255.0)
        )
        drawContours(
            contoursImg,
            yContours,
            Scalar(255.0, 255.0, 0.0),
            Scalar(0.0, 255.0, 255.0)
        )
        drawContours(
            contoursImg,
            bContours,
            Scalar(0.0, 0.0, 255.0),
            Scalar(0.0, 255.0, 255.0)
        )
        val contoursBitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
        Utils.matToBitmap(contoursImg, contoursBitmap)
        lastFrame.set(contoursBitmap)

        return frame
    }

    fun getContours(mask: Mat): Pair<List<MatOfPoint>, Mat> {
        val denoised = mask.clone()
        val denoiseSize = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, Size(10.0, 10.0))
        val denoiseSize2 = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, Size(5.0, 5.0))

        Imgproc.erode(denoised, denoised, denoiseSize)
        Imgproc.dilate(denoised, denoised, denoiseSize)
        Imgproc.erode(denoised, denoised, denoiseSize2)
        Imgproc.morphologyEx(denoised, denoised, Imgproc.MORPH_OPEN, denoiseSize)

        val contours = mutableListOf<MatOfPoint>()
        val hierarchy = Mat()
        Imgproc.findContours(
            denoised,
            contours,
            hierarchy,
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )
        return (contours.toList() to hierarchy)
    }

    fun filterContours(contoursAndHierarchy: Pair<List<MatOfPoint>, Mat>): List<MatOfPoint> {
        return filterContours(contoursAndHierarchy.first, contoursAndHierarchy.second)
    }

    fun filterContours(
        contours: List<MatOfPoint>,
        hierarchy: Mat
    ): List<MatOfPoint> {
        return contours.filterIndexed { i, contour ->
            isValidContour(contour, hierarchy.get(0, i)[3])
        }
    }

    fun isValidContour(
        contour: MatOfPoint,
        hierarchyValue: Double
    ): Boolean {
        if (hierarchyValue != -1.0) {
            return false
        }
        val boundingRect = Imgproc.boundingRect(contour)
        return !(boundingRect.width < 20 || boundingRect.height < 20)
    }

    fun drawContours(
        image: Mat,
        contours: List<MatOfPoint>,
        contourColor: Scalar,
        boxColor: Scalar
    ) {
        Imgproc.drawContours(image, contours, -1, contourColor, 2)
        contours.forEachIndexed { i, contour ->
            val minRect = Imgproc.minAreaRect(MatOfPoint2f(*(contour.toArray())))
            val angle = correctCVAngle(minRect)
            val points = Array<Point>(4) { Point() }
            minRect.points(points)
            Imgproc.line(image, points[0], points[1], boxColor, 2)
            Imgproc.line(image, points[1], points[2], boxColor, 2)
            Imgproc.line(image, points[2], points[3], boxColor, 2)
            Imgproc.line(image, points[3], points[0], boxColor, 2) //code vandalism

            Imgproc.putText(image, "${round(angle)}", minRect.center, Imgproc.FONT_HERSHEY_PLAIN, 4.0, boxColor)
        }
    }

    fun contoursToDetections(contours: List<MatOfPoint>, color: SampleColor): List<SampleDetection> {
        return contours.map { contour -> contourToDetection(contour, color) }
    }

    fun contourToDetection(contour: MatOfPoint, color: SampleColor): SampleDetection {
        val minRect = Imgproc.minAreaRect(MatOfPoint2f(*contour.toArray()))
        val center = Vec2d(minRect.center.x, minRect.center.y) // maybe make an extension function or smt
        // 0 degrees is when the box is fully straight landscape since that's how our intake works

        val angle = correctCVAngle(minRect)

        return SampleDetection(center, angle, color)
    }

    fun correctCVAngle(rect: RotatedRect): Double {
        val angle = if (rect.size.width < rect.size.height) {
            90 - rect.angle
        } else {
            -rect.angle
        }
        val correctedAngle = angle
        return correctedAngle
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

enum class SampleColor {
    RED,
    YELLOW,
    BLUE
}

data class SampleDetection(val pos: Vec2d, val angle: Double, val color: SampleColor)