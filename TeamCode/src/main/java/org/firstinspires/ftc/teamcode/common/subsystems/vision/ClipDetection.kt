package org.firstinspires.ftc.teamcode.common.subsystems.vision

import android.graphics.Bitmap
import android.graphics.Canvas
import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference

@Config
class ClipPipeline : VisionProcessor, CameraStreamSource {
    val lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    val detections: AtomicReference<List<ClipDetection>> = AtomicReference(emptyList())

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(
        frame: Mat,
        captureTimeNanos: Long
    ): Any? {
        val grayscale = Mat()
        Imgproc.cvtColor(frame, grayscale, Imgproc.COLOR_RGB2GRAY)

        // threshold
        val threshold = Mat()
        Imgproc.threshold(grayscale, threshold, ClipPipeline.threshold, 255.0, Imgproc.THRESH_BINARY_INV)

        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(kernalSize, kernalSize))
        // open removes noise, close removes gaps
        Imgproc.morphologyEx(threshold, threshold, Imgproc.MORPH_OPEN, kernel)
        Imgproc.morphologyEx(threshold, threshold, Imgproc.MORPH_CLOSE, kernel)

        val contours = mutableListOf<MatOfPoint>()
        val hierarchy = Mat()
        Imgproc.findContours(
            threshold,
            contours,
            hierarchy,
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        val detections = mutableListOf<ClipDetection>()

        contours.forEach { contour ->
            val minRect = Imgproc.minAreaRect(MatOfPoint2f(*contour.toArray()))

            if (minRect.size.width < minContourSize || minRect.size.height < minContourSize) {
                // too small
                contour.release()
                return@forEach
            }

            val contourArea = Imgproc.contourArea(contour)
            val boundingArea = minRect.size.area()
            if (contourArea / boundingArea < minContourPercent) {
                // likely not a clip, probably a wire,
                // the clip is a rectangle and would occupy a large portion
                // of the bounding box vs a tiny curly wire
                contour.release()
                return@forEach
            }
            Imgproc.drawContours(frame, listOf(contour), -1, Scalar(0.0, 255.0, 0.0), 2)
            contour.release()

            val points = Array<Point>(4) { Point() }
            minRect.points(points)
            val pointMats = Array<MatOfPoint>(4) { MatOfPoint(points[0], points[1], points[2], points[3]) }

            Imgproc.polylines(frame, pointMats.toList(), true, Scalar(0.0, 255.0, 255.0), 2)
            pointMats.forEach { it.release() }

            val center = Vec2d(minRect.center.x, minRect.center.y)
            val size = Vec2d(minRect.size.width, minRect.size.height)

            detections.add(
                ClipDetection(
                    center,
                    minRect.angle,
                    BoundingBox(center, size)
                )
            )
        }

        this.detections.set(detections)

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

    companion object {
        @JvmField
        val threshold: Double = 0.0

        @JvmField
        val kernalSize: Double = 5.0

        @JvmField
        val minContourSize: Double = 20.0

        @JvmField
        val minContourPercent: Double = 0.75
    }
}

data class ClipDetection(
    override val pos: Vec2d,
    override val angle: Double,
    override val boundingBox: BoundingBox
) : Detection