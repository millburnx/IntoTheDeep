package com.millburnx.utils

typealias iVec2d = IVec2d<Number>

/** I can't force ppl to implement all the secondary constructors cleanly,
 * however, please implement for single value, pair, another iVec2d, for all number types
 */
interface IVec2d<T : Number> {
    val x: T
    val y: T

    operator fun plus(other: IVec2d<T>): IVec2d<T>
    operator fun plus(other: IVec2d<Number>): IVec2d<T>
    operator fun plus(other: Number): IVec2d<T>

    operator fun minus(other: IVec2d<T>): IVec2d<T>
    operator fun minus(other: IVec2d<Number>): IVec2d<T>
    operator fun minus(other: Number): IVec2d<T>

    operator fun times(other: IVec2d<T>): IVec2d<T>
    operator fun times(other: IVec2d<Number>): IVec2d<T>
    operator fun times(other: Number): IVec2d<T>

    operator fun div(other: IVec2d<T>): IVec2d<T>
    operator fun div(other: IVec2d<Number>): IVec2d<T>
    operator fun div(other: Number): IVec2d<T>

    operator fun unaryMinus(): IVec2d<T>

    fun toDouble(): IVec2d<Double>
    fun toFloat(): IVec2d<Float>
    fun toInt(): IVec2d<Int>

    fun toDoublePair(): Pair<Double, Double>
    fun toFloatPair(): Pair<Float, Float>
    fun toIntPair(): Pair<Int, Int>

    fun roundToInt(): Pair<Int, Int>
}