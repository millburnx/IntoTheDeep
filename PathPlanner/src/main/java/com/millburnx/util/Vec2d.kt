package com.millburnx.util

import java.io.Serializable
import kotlin.math.cos
import kotlin.math.round
import kotlin.math.sin
import kotlin.math.sqrt

@Suppress("detekt:TooManyFunctions")
public interface IVector2d<T : Number, Self : IVector2d<T, Self>> : Serializable {
    public val x: T
    public val y: T

    public fun distanceTo(other: IVector2d<*, *>): Double
    public fun lerp(other: IVector2d<*, *>, t: Double): Self

    public fun rotate(origin: IVector2d<*, *>, angle: Double): Self
    public fun rotate(angle: Double): IVector2d<T, *>

    public fun toPair(): Pair<T, T>

    public operator fun plus(other: IVector2d<*, *>): Self
    public operator fun plus(other: Number): Self

    public operator fun minus(other: IVector2d<*, *>): Self
    public operator fun minus(other: Number): Self

    public operator fun times(other: IVector2d<*, *>): Self
    public operator fun times(other: Number): Self

    public operator fun div(other: IVector2d<*, *>): Self
    public operator fun div(other: Number): Self

    public operator fun unaryMinus(): Self
}

public typealias Vec2d = Vector2d
public typealias IVec2d = IVector2d<Double, *>

@Suppress("detekt:TooManyFunctions")
public data class Vector2d(override val x: Double = 0.0, override val y: Double = x) : IVector2d<Double, Vector2d> {
    public constructor(vec: IVector2d<*, *>) : this(vec.x.toDouble(), vec.y.toDouble())
    public constructor(pair: Pair<Number, Number>) : this(pair.first.toDouble(), pair.second.toDouble())
    public constructor(x: Number, y: Number = x) : this(x.toDouble(), y.toDouble())

    override fun distanceTo(other: IVector2d<*, *>): Double {
        val dx = x.toDouble() - other.x.toDouble()
        val dy = y.toDouble() - other.y.toDouble()
        return sqrt(dx * dx + dy * dy)
    }

    override fun rotate(origin: IVector2d<*, *>, angle: Double): Vector2d {
        val offset = this - origin
        val cos = cos(angle)
        val sin = sin(angle)
        return Vector2d(
            offset.x * cos - offset.y * sin,
            offset.x * sin + offset.y * cos
        ) + origin
    }

    override fun rotate(angle: Double): Vector2d = rotate(Vector2d(), angle)

    override fun lerp(other: IVector2d<*, *>, t: Double): Vector2d =
        Vector2d(Math.lerp(x, other.x.toDouble(), t), Math.lerp(y, other.y.toDouble(), t))

    override fun toPair(): Pair<Double, Double> = Pair(x, y)
    override fun plus(other: IVector2d<*, *>): Vector2d =
        Vector2d(x + other.x.toDouble(), y + other.y.toDouble())

    override fun plus(other: Number): Vector2d = Vector2d(x + other.toDouble(), y + other.toDouble())
    override fun minus(other: IVector2d<*, *>): Vector2d =
        Vector2d(x - other.x.toDouble(), y - other.y.toDouble())

    override fun minus(other: Number): Vector2d = Vector2d(x - other.toDouble(), y - other.toDouble())
    override fun times(other: IVector2d<*, *>): Vector2d =
        Vector2d(x * other.x.toDouble(), y * other.y.toDouble())

    override fun times(other: Number): Vector2d = Vector2d(x * other.toDouble(), y * other.toDouble())
    override fun div(other: IVector2d<*, *>): Vector2d =
        Vector2d(x / other.x.toDouble(), y / other.y.toDouble())

    override fun div(other: Number): Vector2d = Vector2d(x / other.toDouble(), y / other.toDouble())
    override fun unaryMinus(): Vector2d = Vector2d(-x, -y)
}

public typealias Vec2dF = Vector2dF
public typealias IVec2dF = IVector2d<Float, *>

@Suppress("detekt:TooManyFunctions")
public data class Vector2dF(override val x: Float = 0.0f, override val y: Float = x) : IVector2d<Float, Vector2dF> {
    public constructor(vec: IVector2d<*, *>) : this(vec.x.toFloat(), vec.y.toFloat())
    public constructor(pair: Pair<Number, Number>) : this(pair.first.toFloat(), pair.second.toFloat())
    public constructor(x: Number, y: Number = x) : this(x.toFloat(), y.toFloat())

    override fun distanceTo(other: IVector2d<*, *>): Double {
        val dx = x.toDouble() - other.x.toDouble()
        val dy = y.toDouble() - other.y.toDouble()
        return sqrt(dx * dx + dy * dy)
    }

    override fun rotate(origin: IVector2d<*, *>, angle: Double): Vector2dF {
        val offset = this - origin
        val cos = cos(angle)
        val sin = sin(angle)
        return Vector2dF(
            offset.x * cos - offset.y * sin,
            offset.x * sin + offset.y * cos
        ) + origin
    }

    override fun rotate(angle: Double): Vector2dF = rotate(Vector2dF(), angle)

    override fun lerp(other: IVector2d<*, *>, t: Double): Vector2dF =
        Vector2dF(Math.lerp(x, other.x.toFloat(), t), Math.lerp(y, other.y.toFloat(), t))

    override fun toPair(): Pair<Float, Float> = Pair(x, y)
    override fun plus(other: IVector2d<*, *>): Vector2dF =
        Vector2dF(x + other.x.toFloat(), y + other.y.toFloat())

    override fun plus(other: Number): Vector2dF = Vector2dF(x + other.toFloat(), y + other.toFloat())
    override fun minus(other: IVector2d<*, *>): Vector2dF =
        Vector2dF(x - other.x.toFloat(), y - other.y.toFloat())

    override fun minus(other: Number): Vector2dF = Vector2dF(x - other.toFloat(), y - other.toFloat())
    override fun times(other: IVector2d<*, *>): Vector2dF =
        Vector2dF(x * other.x.toFloat(), y * other.y.toFloat())

    override fun times(other: Number): Vector2dF = Vector2dF(x * other.toFloat(), y * other.toFloat())
    override fun div(other: IVector2d<*, *>): Vector2dF = Vector2dF(x / other.x.toFloat(), y / other.y.toFloat())
    override fun div(other: Number): Vector2dF = Vector2dF(x / other.toFloat(), y / other.toFloat())
    override fun unaryMinus(): Vector2dF = Vector2dF(-x, -y)
}

public typealias Vec2dI = Vector2dI
public typealias IVec2dI = IVector2d<Int, *>

@Suppress("detekt:TooManyFunctions")
public data class Vector2dI(override val x: Int = 0, override val y: Int = x) : IVector2d<Int, Vector2dI> {
    public constructor(vec: IVector2d<*, *>) : this(vec.x.toInt(), vec.y.toInt())
    public constructor(pair: Pair<Number, Number>) : this(pair.first.toInt(), pair.second.toInt())
    public constructor(x: Number, y: Number = x) : this(x.toInt(), y.toInt())

    override fun distanceTo(other: IVector2d<*, *>): Double {
        val dx = x.toDouble() - other.x.toDouble()
        val dy = y.toDouble() - other.y.toDouble()
        return sqrt(dx * dx + dy * dy)
    }

    override fun rotate(origin: IVector2d<*, *>, angle: Double): Vector2dI {
        val offset = this - origin
        val cos = cos(angle)
        val sin = sin(angle)
        return Vector2dI(
            offset.x * cos - offset.y * sin,
            offset.x * sin + offset.y * cos
        ) + origin
    }

    override fun rotate(angle: Double): Vector2dI = rotate(Vector2dI(), angle)

    override fun lerp(other: IVector2d<*, *>, t: Double): Vector2dI =
        Vector2dI(Math.lerp(x, other.x.toInt(), t), Math.lerp(y, other.y.toInt(), t))

    override fun toPair(): Pair<Int, Int> = Pair(x, y)
    override fun plus(other: IVector2d<*, *>): Vector2dI = Vector2dI(x + other.x.toInt(), y + other.y.toInt())
    override fun plus(other: Number): Vector2dI = Vector2dI(x + other.toInt(), y + other.toInt())
    override fun minus(other: IVector2d<*, *>): Vector2dI = Vector2dI(x - other.x.toInt(), y - other.y.toInt())
    override fun minus(other: Number): Vector2dI = Vector2dI(x - other.toInt(), y - other.toInt())
    override fun times(other: IVector2d<*, *>): Vector2dI = Vector2dI(x * other.x.toInt(), y * other.y.toInt())
    override fun times(other: Number): Vector2dI = Vector2dI(x * other.toInt(), y * other.toInt())
    override fun div(other: IVector2d<*, *>): Vector2dI = Vector2dI(x / other.x.toInt(), y / other.y.toInt())
    override fun div(other: Number): Vector2dI = Vector2dI(x / other.toInt(), y / other.toInt())
    override fun unaryMinus(): Vector2dI = Vector2dI(-x, -y)

    public companion object {
        public fun fromRounded(vec: IVector2d<*, *>): Vector2dI =
            Vector2dI(round(vec.x.toDouble()), round(vec.y.toDouble()))

        public fun fromRounded(pair: Pair<Number, Number>): Vector2dI =
            Vector2dI(round(pair.first.toDouble()), round(pair.second.toDouble()))

        public fun fromRounded(x: Number, y: Number = x): Vector2dI =
            Vector2dI(round(x.toDouble()), round(y.toDouble()))
    }
}

