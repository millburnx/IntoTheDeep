package com.millburnx.util.geometry.basic.vector

import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

@Suppress("detekt:TooManyFunctions")
public data class Vec2d(override val x: Double, override val y: Double) : Vec2<Double, Vec2d> {
    public constructor(x: Number = 0, y: Number = x) : this(x.toDouble(), y.toDouble())
    public constructor(vec: Vec2<*, *>) : this(vec.x, vec.y)
    public constructor(pair: Pair<Number, Number>) : this(pair.first.toDouble(), pair.second.toDouble())
    public constructor(list: List<Number>) : this(list[0].toDouble(), list[1].toDouble())
    public constructor(array: Array<Number>) : this(array[0].toDouble(), array[1].toDouble())

    override fun plus(other: Vec2<*, *>): Vec2d =
        Vec2d(x + other.x.toDouble(), y + other.y.toDouble())

    override fun plus(other: Number): Vec2d =
        Vec2d(x + other.toDouble(), y + other.toDouble())

    override fun minus(other: Vec2<*, *>): Vec2d =
        Vec2d(x - other.x.toDouble(), y - other.y.toDouble())

    override fun minus(other: Number): Vec2d =
        Vec2d(x - other.toDouble(), y - other.toDouble())

    override fun times(other: Vec2<*, *>): Vec2d =
        Vec2d(x * other.x.toDouble(), y * other.y.toDouble())

    override fun times(other: Number): Vec2d =
        Vec2d(x * other.toDouble(), y * other.toDouble())

    override fun div(other: Vec2<*, *>): Vec2d =
        Vec2d(x / other.x.toDouble(), y / other.y.toDouble())

    override fun div(other: Number): Vec2d =
        Vec2d(x / other.toDouble(), y / other.toDouble())

    override fun unaryMinus(): Vec2d =
        Vec2d(-x, -y)

    override fun pow(other: Vec2<*, *>): Vec2d =
        Vec2d(x.pow(other.x.toDouble()), y.pow(other.y.toDouble()))

    override fun pow(other: Number): Vec2d =
        Vec2d(x.pow(other.toDouble()), y.pow(other.toDouble()))

    override fun reciprocal(): Vec2d =
        Vec2d(
            if (x == 0.0) Double.POSITIVE_INFINITY else 1.0 / x,
            if (y == 0.0) Double.POSITIVE_INFINITY else 1.0 / y
        )

    override fun abs(): Vec2d =
        Vec2d(x.absoluteValue, y.absoluteValue)

    override fun sum(): Double = x + y
    override fun magnitude(): Double = sqrt(this.dot(this))
    override fun normalize(): Vec2d = this / this.magnitude()
    override fun distance(other: Vec2<*, *>): Double = sqrt((-this + other).pow(2.0).sum())
    override fun dot(other: Vec2<*, *>): Double = x * other.x.toDouble() + y * other.y.toDouble()
    override fun cross(other: Vec2<*, *>): Double = x * other.y.toDouble() - y * other.x.toDouble()
    override fun angle(other: Vec2<*, *>): Double = atan2(other.y.toDouble(), other.x.toDouble()) - atan2(y, x)
    override fun project(other: Vec2<*, *>): Vec2d =
        this * other / other.magnitude()

    override fun lerp(other: Vec2<*, *>, t: Number): Vec2d =
        this + (-this + other) * t.toDouble()

    override fun rotate(angle: Number): Vec2d =
        Vec2d(
            x * cos(angle.toDouble()) - y * sin(angle.toDouble()),
            x * sin(angle.toDouble()) + y * cos(angle.toDouble())
        )

    override fun perpendicular(): Vec2d =
        Vec2d(-y, x)

    override fun toDouble(): Vec2<Double, *> = Vec2d(x, y)
    override fun toFloat(): Vec2<Float, *> = Vec2f(x, y)

    override fun toInt(): Vec2<Int, *> {
        TODO("Not yet implemented")
    }

    override fun roundToInt(): Vec2<Int, *> {
        TODO("Not yet implemented")
    }

    override fun toPair(): Pair<Double, Double> = x to y

    override fun toList(): List<Double> = listOf(x, y)

    override fun toArray(): Array<Double> = arrayOf(x, y)

    public companion object {
        public fun fromRR(vec: Vec2d): Vec2d = Vec2d(vec.x.toFloat(), vec.y.toFloat())
    }
}
