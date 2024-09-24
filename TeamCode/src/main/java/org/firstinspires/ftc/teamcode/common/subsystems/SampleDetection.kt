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
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference

class SampleDetection : SubsystemBase() {

}

@Config
object SampleDetectorConfig {
    @JvmField
    var color = 0
}

class SampleDetector : VisionProcessor, CameraStreamSource {
    val lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

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

        val (rContours, rHierarchy) = getContours(rThreshold)
        val (yContours, yHierarchy) = getContours(yThreshold)
        val (bContours, bHierarchy) = getContours(bThreshold)

        val contoursImg = frame.clone()
        drawContours(
            contoursImg,
            rContours,
            rHierarchy,
            Scalar(255.0, 0.0, 0.0),
            Scalar(0.0, 255.0, 255.0)
        )
        drawContours(
            contoursImg,
            yContours,
            yHierarchy,
            Scalar(255.0, 255.0, 0.0),
            Scalar(0.0, 255.0, 255.0)
        )
        drawContours(
            contoursImg,
            bContours,
            bHierarchy,
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

    fun drawContours(
        image: Mat,
        contours: List<MatOfPoint>,
        hierarchy: Mat,
        contourColor: Scalar,
        boxColor: Scalar
    ) {
        Imgproc.drawContours(image, contours, -1, contourColor, 2)
        println("processing ${contours.size} contours")
        contours.forEachIndexed { i, contour ->
            if (hierarchy.get(0, i)[3] != -1.0) {
                return@forEachIndexed
            }
            val boundingRect = Imgproc.boundingRect(contour)
            if (boundingRect.width < 20 || boundingRect.height < 20) {
                println("discarded contour $i , (${boundingRect.width} x ${boundingRect.height})")
                return@forEachIndexed
            }
            val minRect = Imgproc.minAreaRect(MatOfPoint2f(*contour.toArray()))
            val points = Array<Point>(4) { Point() }
            minRect.points(points)
            println(points.joinToString(", "))
            Imgproc.line(image, points[0], points[1], boxColor, 2)
            Imgproc.line(image, points[1], points[2], boxColor, 2)
            Imgproc.line(image, points[2], points[3], boxColor, 2)
            Imgproc.line(image, points[3], points[0], boxColor, 2) //code vandalism

        }
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        TODO("Not yet implemented")
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>) {
        continuation.dispatch { consumer -> consumer.accept(lastFrame.get()) }
    }

}