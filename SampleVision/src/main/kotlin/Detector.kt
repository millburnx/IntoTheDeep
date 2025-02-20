package org.example

import com.millburnx.utils.Vec2d
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import org.opencv.imgproc.Imgproc
import kotlin.math.max
import kotlin.math.min

public class RotRect(
    public val rotatedRect: RotatedRect,
) {
    public val angle: Double = rotatedRect.angle
    public val size: Vec2d = Vec2d(rotatedRect.size.width, rotatedRect.size.height)
    public val area: Double = size.x * size.y
    public val center: Vec2d = Vec2d(rotatedRect.center.x, rotatedRect.center.y)
    public val points: List<Vec2d>

    init {
        val pointsArr: Array<Point> = arrayOf(Point(), Point(), Point(), Point())
        rotatedRect.points(pointsArr)
        points = pointsArr.map { Vec2d(it.x, it.y) }
    }
}

public data class Sample(
    val color: Color?,
    val center: Vec2d,
    val minBounding: RotRect,
    val adjustedCenter: Vec2d?,
) {
    public enum class Color {
        YELLOW {
            override val channel: Int = 2
            override val range: ClosedRange<Double> = 90.0..255.0
            override val invertedRange: Boolean = true
        },
        BLUE {
            override val channel: Int = 2
            override val range: ClosedRange<Double> = 150.0..255.0
        },
        RED {
            override val channel: Int = 1
            override val range: ClosedRange<Double> = 199.0..255.0
        }, ;

        public abstract val channel: Int
        public abstract val range: ClosedRange<Double>
        public open val invertedRange: Boolean = false
    }

    public fun setColor(color: Color): Sample = Sample(color, center, minBounding, adjustedCenter)
}

public class Detector(
    public val colors: List<Sample.Color>,
    public val areaThreshold: ClosedRange<Double>,
) {
    private fun detectMultiSamplesInternal(ycrcbFrame: Mat): Map<Sample.Color, List<Sample>> =
        colors.associateWith { detectSamplesInternal(ycrcbFrame, it) }

    private fun detectSamplesInternal(
        ycrcbFrame: Mat,
        color: Sample.Color,
    ): List<Sample> {
        val maskedFrame = sampleThreshold(ycrcbFrame, color)
        val contours = getSampleContours(maskedFrame)
        val samples = getSamples(ycrcbFrame, contours)
        return samples
    }

    private fun sampleThreshold(
        ycrcbFrame: Mat,
        color: Sample.Color,
    ): Mat {
        val channels: MutableList<Mat> = mutableListOf()
        Core.split(ycrcbFrame, channels)
        val channel = channels[color.channel]
        val threshold = Mat()
        Imgproc.threshold(
            channels[1],
            threshold,
            color.range.start,
            color.range.endInclusive,
            if (color.invertedRange) Imgproc.THRESH_BINARY_INV else Imgproc.THRESH_BINARY,
        )
        return threshold
    }

    private fun getSampleContours(maskedFrame: Mat): List<MatOfPoint> {
        val contours: MutableList<MatOfPoint> = mutableListOf()
        val hierarchy = Mat()
        Imgproc.findContours(
            maskedFrame,
            contours,
            hierarchy,
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_SIMPLE,
        )
        return contours
    }

    // provide ppi(pixels per inch) if you want the pipeline to try and fix samples located on the edge of the frame
    private fun getSamples(
        frame: Mat,
        contours: List<MatOfPoint>,
        ppi: Double? = null,
    ): List<Sample> = contours.map { getSample(frame, it, ppi) }.filterNotNull()

    private fun getSample(
        frame: Mat,
        contour: MatOfPoint,
        ppi: Double? = null,
    ): Sample? {
        val minBounding = RotRect(Imgproc.minAreaRect(MatOfPoint2f(*contour.toArray())))
        if (!areaThreshold.contains(minBounding.area)) return null // too small/large a contour (mostly too small)
        if (ppi == null) {
            return Sample(
                null,
                minBounding.center,
                minBounding,
                null,
            )
        }
        // check if the center is near the edge and the area/sides are smal`ler than supposed to be
        // samples are 1.5x3.5 inches, adjust down a bit to account for goofy contours
        val longestSide = max(minBounding.size.x, minBounding.size.y)
        val shortestSide = min(minBounding.size.x, minBounding.size.y)
        return Sample(null, minBounding.center, minBounding, null)
    }
}
g