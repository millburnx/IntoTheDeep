package com.millburnx.util.geometry.basic.vector

import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt

@Suppress("detekt:TooManyFunctions")
// All the operations are like floats or just like bad
// it's really only good for holding values, not actually for operations
public data class Vec2i(override val x: Int, override val y: Int) : Vec2<Int, Vec2i> {
    public constructor(x: Number = 0, y: Number = x) : this(x.toInt(), y.toInt())
    public constructor(vec: Vec2<*, *>) : this(vec.x, vec.y)
    public constructor(pair: Pair<Number, Number>) : this(pair.first.toInt(), pair.second.toInt())
    public constructor(list: List<Number>) : this(list[0].toInt(), list[1].toInt())
    public constructor(array: Array<Number>) : this(array[0].toInt(), array[1].toInt())

    override fun plus(other: Vec2<*, *>): Vec2i = Vec2i(x + other.x.toInt(), y + other.y.toInt())
    override fun plus(other: Number): Vec2i = Vec2i(x + other.toInt(), y + other.toInt())
    override fun minus(other: Vec2<*, *>): Vec2i = Vec2i(x - other.x.toInt(), y - other.y.toInt())
    override fun minus(other: Number): Vec2i = Vec2i(x - other.toInt(), y - other.toInt())
    override fun times(other: Vec2<*, *>): Vec2i = Vec2i(x * other.x.toInt(), y * other.y.toInt())
    override fun times(other: Number): Vec2i = Vec2i(x * other.toInt(), y * other.toInt())
    override fun div(other: Vec2<*, *>): Vec2i = Vec2i(x / other.x.toInt(), y / other.y.toInt())
    override fun div(other: Number): Vec2i = Vec2i(x / other.toInt(), y / other.toInt())
    override fun unaryMinus(): Vec2i = Vec2i(-x, -y)
    override fun pow(other: Vec2<*, *>): Vec2i =
        Vec2i(x.toFloat().pow(other.x.toFloat()), y.toFloat().pow(other.y.toInt()))

    override fun pow(other: Number): Vec2i = Vec2i(x.toFloat().pow(other.toInt()), y.toFloat().pow(other.toInt()))
    public fun powRounded(other: Number): Vec2i =
        Vec2i(x.toFloat().pow(other.toInt()).roundToInt(), y.toFloat().pow(other.toInt()).roundToInt())

    override fun reciprocal(): Vec2i =
        Vec2i(if (x == 0) Int.MAX_VALUE else 1 / x, if (y == 0) Int.MAX_VALUE else 1 / y)

    override fun abs(): Vec2i = Vec2i(x.absoluteValue, y.absoluteValue)
    override fun sum(): Int = x + y
    override fun magnitude(): Float = sqrt(this.dot(this).toFloat()).toFloat()
    override fun normalize(): Vec2i = this / this.magnitude()
    override fun distance(other: Vec2<*, *>): Float = sqrt((-this + other).pow(2.0).sum().toFloat())
    override fun dot(other: Vec2<*, *>): Int = (x * other.x.toFloat() + y * other.y.toFloat()).toInt()
    public fun dotRound(other: Vec2<*, *>): Int = (x * other.x.toFloat() + y * other.y.toFloat()).roundToInt()
    override fun cross(other: Vec2<*, *>): Int = (x * other.y.toFloat() - y * other.x.toFloat()).toInt()
    public fun crossRound(other: Vec2<*, *>): Int = (x * other.y.toFloat() - y * other.x.toFloat()).roundToInt()
    override fun angle(other: Vec2<*, *>): Float =
        (atan2(other.y.toFloat(), other.x.toFloat()) - atan2(y.toFloat(), x.toFloat()))

    override fun project(other: Vec2<*, *>): Vec2i = this * other / other.magnitude()
    override fun lerp(other: Vec2<*, *>, t: Number): Vec2i = this + (-this + other) * t.toDouble()
    override fun rotate(angle: Number): Vec2i =
        Vec2i(x * cos(angle.toFloat()) - y * sin(angle.toFloat()), x * sin(angle.toFloat()) + y * cos(angle.toFloat()))

    override fun perpendicular(): Vec2i = Vec2i(-y, x)
    override fun toDouble(): Vec2<Double, *> = com.millburnx.util.geometry.basic.vector.Vec2d(x, y)
    override fun toFloat(): Vec2<Float, *> = Vec2f(x, y)
    override fun toInt(): Vec2<Int, *> = Vec2i(x, y)

    override fun roundToInt(): Vec2<Int, *> = Vec2i(x.toDouble().roundToInt(), y.toDouble().roundToInt())

    override fun toPair(): Pair<Int, Int> = x to y
    override fun toList(): List<Int> = listOf(x, y)
    override fun toArray(): Array<Int> = arrayOf(x, y)
}
