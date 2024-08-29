package com.millburnx.utils.vec2d

import com.millburnx.utils.IVec2d
import kotlin.math.roundToInt

data class Vec2d(override val x: Double, override val y: Double) : IVec2d<Double> {
    constructor(vec2d: IVec2d<Number>) : this(vec2d.x.toDouble(), vec2d.y.toDouble())
    constructor(pair: Pair<Number, Number>) : this(pair.first, pair.second)
    constructor(x: Number, y: Number) : this(x.toDouble(), y.toDouble())
    constructor(v: Number) : this(v.toDouble(), v.toDouble())

    override fun <Y : Number> plus(other: IVec2d<Y>): Vec2d = Vec2d(x + other.x.toDouble(), y + other.y.toDouble())
    override fun plus(other: Number): Vec2d = Vec2d(x + other.toDouble(), y + other.toDouble())

    override fun <Y : Number> minus(other: IVec2d<Y>): Vec2d = Vec2d(x - other.x.toDouble(), y - other.y.toDouble())
    override fun minus(other: Number): Vec2d = Vec2d(x - other.toDouble(), y - other.toDouble())

    override fun <Y : Number> times(other: IVec2d<Y>): Vec2d = Vec2d(x * other.x.toDouble(), y * other.y.toDouble())
    override fun times(other: Number): Vec2d = Vec2d(x * other.toDouble(), y * other.toDouble())

    override fun <Y : Number> div(other: IVec2d<Y>): Vec2d = Vec2d(x / other.x.toDouble(), y / other.y.toDouble())
    override fun div(other: Number): Vec2d = Vec2d(x / other.toDouble(), y / other.toDouble())

    override fun unaryMinus(): Vec2d = Vec2d(-x, -y)

    override fun toDouble(): Vec2d = this.copy()
    override fun toFloat(): IVec2d<Float> = TODO("Not yet implemented")
    override fun toInt(): Vec2dInt = Vec2dInt(x.toInt(), y.toInt())

    override fun toDoublePair(): Pair<Double, Double> = Pair(x, y)
    override fun toFloatPair(): Pair<Float, Float> = Pair(x.toFloat(), y.toFloat())
    override fun toIntPair(): Pair<Int, Int> = Pair(x.toInt(), y.toInt())

    override fun roundToInt(): Pair<Int, Int> = Pair(x.roundToInt(), y.roundToInt())
}