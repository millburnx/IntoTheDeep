package com.millburnx.utils

import com.millburnx.utils.vec2d.Vec2d
import com.millburnx.utils.vec2d.Vec2dInt
import kotlin.math.roundToInt

data class Scale(val ppi: Double, val scale: Double) {
    fun ppiFractionalScale(value: Vec2d): Vec2d = value * ppi
    fun ppiFractionalScale(value: Vec2dInt): Vec2dInt = value * ppi

    fun ppiFractionalScale(pair: Pair<Double, Double>): Pair<Double, Double> = Pair(pair.first * ppi, pair.second * ppi)
    fun ppiFractionalScale(pair: Pair<Float, Float>): Pair<Double, Double> = Pair(pair.first * ppi, pair.second * ppi)
    fun ppiFractionalScale(pair: Pair<Int, Int>): Pair<Double, Double> = Pair(pair.first * ppi, pair.second * ppi)

    fun ppiFractionalScale(value: Double): Double = value * ppi
    fun ppiFractionalScale(value: Float): Double = value * ppi
    fun ppiFractionalScale(value: Int): Double = value * ppi

    fun ppiScale(vec2d: Vec2d): Vec2dInt = Vec2dInt((vec2d * ppi * scale).roundToInt())
    fun ppiScale(vec2d: Vec2dInt): Vec2dInt = Vec2dInt((vec2d * ppi * scale).roundToInt())

    fun ppiScale(pair: Pair<Double, Double>): Pair<Int, Int> = Pair(ppiScale(pair.first), ppiScale(pair.second))
    fun ppiScale(pair: Pair<Float, Float>): Pair<Int, Int> = Pair(ppiScale(pair.first), ppiScale(pair.second))
    fun ppiScale(pair: Pair<Int, Int>): Pair<Int, Int> = Pair(ppiScale(pair.first), ppiScale(pair.second))

    fun ppiScale(value: Double): Int = (value * ppi * scale).roundToInt()
    fun ppiScale(value: Float): Int = (value * ppi * scale).roundToInt()
    fun ppiScale(value: Int): Int = (value * ppi * scale).roundToInt()
}