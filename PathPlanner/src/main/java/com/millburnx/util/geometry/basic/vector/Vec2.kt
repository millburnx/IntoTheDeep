package com.millburnx.util.geometry.basic.vector

import java.io.Serializable

@Suppress("detekt:TooManyFunctions")
public interface Vec2<T : Number, Self : Vec2<T, Self>> : Serializable {
    public val x: T
    public val y: T

    // default operators
    public operator fun plus(other: Vec2<*, *>): Self
    public operator fun plus(other: Number): Self
    public operator fun minus(other: Vec2<*, *>): Self
    public operator fun minus(other: Number): Self
    public operator fun times(other: Vec2<*, *>): Self
    public operator fun times(other: Number): Self
    public operator fun div(other: Vec2<*, *>): Self
    public operator fun div(other: Number): Self
    public operator fun unaryMinus(): Self

    // custom operators
    public fun pow(other: Vec2<*, *>): Self
    public fun pow(other: Number): Self
    public fun reciprocal(): Self
    public fun abs(): Self
    public fun sum(): T
    public fun magnitude(): Number
    public fun normalize(): Self

    // custom functions
    public fun distance(other: Vec2<*, *>): Number
    public fun dot(other: Vec2<*, *>): T
    public fun cross(other: Vec2<*, *>): T
    public fun angle(other: Vec2<*, *>): Number
    public fun project(other: Vec2<*, *>): Self

    public fun lerp(other: Vec2<*, *>, t: Number): Self

    // solo functions
    public fun rotate(angle: Number): Self
    public fun perpendicular(): Self

    // conversion functions
    public fun toDouble(): Vec2<Double, *>
    public fun toFloat(): Vec2<Float, *>
    public fun toInt(): Vec2<Int, *>

    public fun roundToInt(): Vec2<Int, *>

    public fun toPair(): Pair<T, T>
    public fun toList(): List<T>
    public fun toArray(): Array<T>
}

public operator fun Number.plus(vec: Vec2<*, *>): Vec2<*, *> = vec + this
public operator fun Number.minus(vec: Vec2<*, *>): Vec2<*, *> = -vec + this
public operator fun Number.div(vec: Vec2<*, *>): Vec2<*, *> = vec.reciprocal() * this
public operator fun Number.times(vec: Vec2<*, *>): Vec2<*, *> = vec * this
