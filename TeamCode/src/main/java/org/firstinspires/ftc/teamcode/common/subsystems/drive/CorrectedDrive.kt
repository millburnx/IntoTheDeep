package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.toCorrectedVec2d
import kotlin.math.pow

@Config
class CorrectedDrive(robot: Robot) : Drive(robot) {

    companion object {
        @JvmField
        var centripetalWeight: Double = 0.1
    }

    val positionBuffer = RingBuffer<Vec2d>(3)

    override fun fieldCentric(x: Double, y: Double, rotate: Double, heading: Double) {
        val centripetalCorrection = getCentripetalCorrection()
        super.fieldCentric(x + centripetalCorrection.x, y + centripetalCorrection.y, rotate, heading)
    }

    // https://en.wikipedia.org/wiki/Circumcircle#Circumcenter_coordinates
    fun getCircumcenter(): Vec2d {
        if (!positionBuffer.isFull) return Vec2d(0)
        val p = positionBuffer.values
        val a = p[0]
        val b = p[1] - a
        val c = p[2] - a
        val d = 2 * (b.x * c.y - b.y * c.x)
        if (d == 0.0) return Vec2d(0)
        val uX = 1 / d * (c.y * b.dot(b) - b.y * c.dot(c))
        val uY = 1 / d * (b.x * c.dot(c) - c.x * b.dot(b))
        return Vec2d(uX, uY) + a
    }

    fun getCentripetalCorrection(
        currentVelocity: Vec2d = odometry.poseVelocity?.toCorrectedVec2d() ?: Vec2d(0),
        weighting: Double = centripetalWeight
    ): Vec2d {
        if (!positionBuffer.isFull) return Vec2d(0)
        val current = positionBuffer.get(2)!!
        val circumcenter = getCircumcenter()

        val centralVector = circumcenter - current
        val centripetalVector = centralVector / centralVector.dot(centralVector)
        val power = weighting * centripetalVector.normalize() * currentVelocity.normalize().pow(2)

        return centripetalVector / centripetalVector.normalize() * power
    }
}

class RingBuffer<T>(val size: Int) {
    private val buffer: MutableList<T> = mutableListOf()

    val values: List<T>
        get() = buffer.toList()

    val isFull: Boolean
        get() = buffer.size == size

    fun add(value: T) {
        if (isFull) {
            buffer.removeAt(0)
        }
        buffer.add(value)
    }

    fun get(index: Int): T? {
        return buffer.getOrNull(index)
    }
}