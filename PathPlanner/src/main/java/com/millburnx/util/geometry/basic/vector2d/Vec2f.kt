package com.millburnx.util.geometry.basic.vector2d

import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

@Suppress("detekt:TooManyFunctions")
public data class Vec2f(override val x: Float, override val y: Float) : Vec2<Float, Vec2f> {
    public constructor(x: Number = 0, y: Number = x) : this(x.toFloat(), y.toFloat())
    public constructor(vec: Vec2<*, *>) : this(vec.x, vec.y)
    public constructor(pair: Pair<Number, Number>) : this(pair.first.toFloat(), pair.second.toFloat())
    public constructor(list: List<Number>) : this(list[0].toFloat(), list[1].toFloat())
    public constructor(array: Array<Number>) : this(array[0].toFloat(), array[1].toFloat())

    override fun plus(other: Vec2<*, *>): Vec2f = Vec2f(x + other.x.toFloat(), y + other.y.toFloat())
    override fun plus(other: Number): Vec2f = Vec2f(x + other.toFloat(), y + other.toFloat())
    override fun minus(other: Vec2<*, *>): Vec2f = Vec2f(x - other.x.toFloat(), y - other.y.toFloat())
    override fun minus(other: Number): Vec2f = Vec2f(x - other.toFloat(), y - other.toFloat())
    override fun times(other: Vec2<*, *>): Vec2f = Vec2f(x * other.x.toFloat(), y * other.y.toFloat())
    override fun times(other: Number): Vec2f = Vec2f(x * other.toFloat(), y * other.toFloat())
    override fun div(other: Vec2<*, *>): Vec2f = Vec2f(x / other.x.toFloat(), y / other.y.toFloat())
    override fun div(other: Number): Vec2f = Vec2f(x / other.toFloat(), y / other.toFloat())
    override fun unaryMinus(): Vec2f = Vec2f(-x, -y)
    override fun pow(other: Vec2<*, *>): Vec2f = Vec2f(x.pow(other.x.toFloat()), y.pow(other.y.toFloat()))
    override fun pow(other: Number): Vec2f = Vec2f(x.pow(other.toFloat()), y.pow(other.toFloat()))
    override fun reciprocal(): Vec2f =
        Vec2f(if (x == 0f) Float.POSITIVE_INFINITY else 1f / x, if (y == 0f) Float.POSITIVE_INFINITY else 1f / y)

    override fun abs(): Vec2f = Vec2f(x.absoluteValue, y.absoluteValue)
    override fun sum(): Float = x + y
    override fun magnitude(): Float = sqrt(this.dot(this))
    override fun normalize(): Vec2f = this / this.magnitude()
    override fun distance(other: Vec2<*, *>): Float = sqrt((-this + other).pow(2.0).sum())
    override fun dot(other: Vec2<*, *>): Float = x * other.x.toFloat() + y * other.y.toFloat()
    override fun cross(other: Vec2<*, *>): Float = x * other.y.toFloat() - y * other.x.toFloat()
    override fun angle(other: Vec2<*, *>): Float = atan2(other.y.toFloat(), other.x.toFloat()) - atan2(y, x)
    override fun project(other: Vec2<*, *>): Vec2f = this * other / other.magnitude()
    override fun lerp(other: Vec2<*, *>, t: Number): Vec2f = this + (-this + other) * t.toDouble()
    override fun rotate(angle: Number): Vec2f =
        Vec2f(x * cos(angle.toFloat()) - y * sin(angle.toFloat()), x * sin(angle.toFloat()) + y * cos(angle.toFloat()))

    override fun perpendicular(): Vec2f = Vec2f(-y, x)
    override fun toDouble(): Vec2<Double, *> = com.millburnx.util.geometry.basic.vector2d.Vec2d(x, y)
    override fun toFloat(): Vec2<Float, *> = Vec2f(x, y)

    override fun toInt(): Vec2<Int, *> {
        TODO("Not yet implemented")
    }

    override fun roundToInt(): Vec2<Int, *> {
        TODO("Not yet implemented")
    }

    override fun toPair(): Pair<Float, Float> = x to y

    override fun toList(): List<Float> = listOf(x, y)

    override fun toArray(): Array<Float> = arrayOf(x, y)
}
