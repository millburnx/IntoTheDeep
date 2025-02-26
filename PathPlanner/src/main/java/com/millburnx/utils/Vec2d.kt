package com.millburnx.utils

import java.awt.Dimension
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Represents a 2D vector/point
 */
data class Vec2d(
    val x: Double = 0.0,
    val y: Double = x,
) {
    constructor(x: Number, y: Number) : this(x.toDouble(), y.toDouble())

    constructor(v: Double) : this(v, v)
    constructor(v: Float) : this(v.toDouble())
    constructor(v: Int) : this(v.toDouble())

    constructor(point: java.awt.Point) : this(point.x.toDouble(), point.y.toDouble())
    constructor(size: Dimension) : this(size.width, size.height)

    operator fun plus(other: Vec2d) = Vec2d(x + other.x, y + other.y)

    operator fun plus(other: Double) = Vec2d(x + other, y + other)

    operator fun plus(other: Float) = this + other.toDouble()

    operator fun plus(other: Int) = this + other.toDouble()

    operator fun minus(other: Vec2d) = Vec2d(x - other.x, y - other.y)

    operator fun minus(other: Double) = Vec2d(x - other, y - other)

    operator fun minus(other: Float) = this - other.toDouble()

    operator fun minus(other: Int) = this - other.toDouble()

    operator fun times(other: Vec2d) = Vec2d(x * other.x, y * other.y)

    operator fun times(other: Double) = Vec2d(x * other, y * other)

    operator fun times(other: Float) = this * other.toDouble()

    operator fun times(other: Int) = this * other.toDouble()

    operator fun div(other: Vec2d) = Vec2d(x / other.x, y / other.y)

    operator fun div(other: Double) = Vec2d(x / other, y / other)

    operator fun div(other: Float) = this / other.toDouble()

    operator fun div(other: Int) = this / other.toDouble()

    operator fun unaryMinus() = Vec2d(-x, -y)

    /**
     * Returns the Euclidean distance to another point
     */
    fun distanceTo(other: Vec2d): Double {
        val xDiff = x - other.x
        val yDiff = y - other.y
        return sqrt(xDiff * xDiff + yDiff * yDiff)
    }

    fun normalize(): Double = sqrt(this.dot(this))

    fun dot(other: Vec2d): Double = x * other.x + y * other.y

    /**
     * Returns the angle to another point
     */
    fun angleTo(other: Vec2d): Double = atan2(other.y - y, other.x - x)

    override fun toString(): String = "Point($x, $y)"

    /**
     * Rotates the vector by an angle
     */
    fun rotate(angle: Double): Vec2d {
        val cos = cos(angle)
        val sin = sin(angle)
        return Vec2d(x * cos - y * sin, x * sin + y * cos)
    }

    /**
     * Linearly interpolates between two vectors/points
     */
    fun lerp(
        other: Vec2d,
        t: Double,
    ) = this + (other - this) * t

    fun lerp(
        other: Vec2d,
        t: Float,
    ) = this.lerp(other, t.toDouble())

    /**
     * Returns a copy of the vector with the absolute value of each component
     */
    fun abs() = Vec2d(abs(x), abs(y))

    fun sqrt() = Vec2d(sqrt(x), sqrt(y))

    fun sign() = Vec2d(sign(x), sign(y))

    fun coerceIn(
        min: Vec2d,
        max: Vec2d,
    ) = Vec2d(x.coerceIn(min.x, max.x), y.coerceIn(min.y, max.y))

    fun coerceIn(
        min: Double,
        max: Double,
    ) = coerceIn(Vec2d(min), Vec2d(max))

    /**
     * Converts the point to a java.awt point
     */
    fun awt(): java.awt.Point = java.awt.Point(x.toInt(), y.toInt())

    /**
     * Converts the point to a java.awt inset
     */
    fun insets(): java.awt.Insets = java.awt.Insets(y.toInt(), x.toInt(), y.toInt(), x.toInt())

    fun flip() = Vec2d(y, x)

    /**
     * Converts the point to a java.awt dimension
     */
    fun dimension(): Dimension = Dimension(x.toInt(), y.toInt())

    fun toRR(): Vec2d = Vec2d(y, -x)

    companion object {
        fun fromRR(point: Vec2d): Vec2d = Vec2d(-point.y, point.x)

        fun fromAngle(radians: Double) = Vec2d(cos(radians), sin(radians))

        /**
         * Saves a list of points to a tsv file
         */
        fun saveList(
            points: List<Vec2d>,
            file: java.io.File,
        ) {
            val data = points.map { listOf(it.x.toString(), it.y.toString()) }
            TSV.bufferedWrite(file, data)
        }

        /**
         * Loads a list of points from a tsv file
         */
        fun loadList(file: java.io.File): Path {
            val data = TSV.bufferedRead(file)
            var endHeading: Double? = null
            val points: MutableList<Vec2d> = mutableListOf()
            for (item in data) {
                if (item.size == 1) {
                    endHeading = item[0].toDouble()
                } else {
                    points.add(Vec2d(item[0].toDouble(), item[1].toDouble()))
                }
            }
            return Path(points, endHeading)
        }
    }
}

data class Path(
    val points: List<Vec2d>,
    val endHeading: Double? = null,
) {
    companion object
}
