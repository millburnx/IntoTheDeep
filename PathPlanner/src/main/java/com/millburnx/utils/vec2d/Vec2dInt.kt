package com.millburnx.utils.vec2d

import com.millburnx.utils.IVec2d
import com.millburnx.utils.iVec2d

data class Vec2dInt(override val x: Int, override val y: Int) : IVec2d<Int> {
    constructor(vec2d: iVec2d) : this(vec2d.x.toInt(), vec2d.y.toInt())
    constructor(pair: Pair<Number, Number>) : this(pair.first, pair.second)
    constructor(x: Number, y: Number) : this(x.toInt(), y.toInt())
    constructor(v: Number) : this(v.toInt(), v.toInt())

    override fun plus(other: IVec2d<Int>): Vec2dInt = Vec2dInt(x + other.x.toInt(), y + other.y.toInt())
    override fun plus(other: iVec2d): Vec2dInt = Vec2dInt(x + other.x.toInt(), y + other.y.toInt())
    override fun plus(other: Number): Vec2dInt = Vec2dInt(x + other.toInt(), y + other.toInt())

    override fun minus(other: IVec2d<Int>): Vec2dInt = Vec2dInt(x - other.x.toInt(), y - other.y.toInt())
    override fun minus(other: iVec2d): Vec2dInt = Vec2dInt(x - other.x.toInt(), y - other.y.toInt())
    override fun minus(other: Number): Vec2dInt = Vec2dInt(x - other.toInt(), y - other.toInt())

    override fun times(other: IVec2d<Int>): Vec2dInt = Vec2dInt(x * other.x.toInt(), y * other.y.toInt())
    override fun times(other: iVec2d): Vec2dInt = Vec2dInt(x * other.x.toInt(), y * other.y.toInt())
    override fun times(other: Number): Vec2dInt = Vec2dInt(x * other.toInt(), y * other.toInt())

    override fun div(other: IVec2d<Int>): Vec2dInt = Vec2dInt(x / other.x.toInt(), y / other.y.toInt())
    override fun div(other: iVec2d): Vec2dInt = Vec2dInt(x / other.x.toInt(), y / other.y.toInt())
    override fun div(other: Number): Vec2dInt = Vec2dInt(x / other.toInt(), y / other.toInt())

    override fun unaryMinus(): Vec2dInt = Vec2dInt(-x, -y)

    override fun toDouble(): Vec2d = Vec2d(x.toDouble(), y.toDouble())
    override fun toFloat(): IVec2d<Float> = TODO("Not yet implemented")
    override fun toInt(): Vec2dInt = this

    override fun toDoublePair(): Pair<Double, Double> = Pair(x.toDouble(), y.toDouble())
    override fun toFloatPair(): Pair<Float, Float> = Pair(x.toFloat(), y.toFloat())
    override fun toIntPair(): Pair<Int, Int> = Pair(x.toInt(), y.toInt())

    override fun roundToInt(): Pair<Int, Int> = Pair(x, y)
}