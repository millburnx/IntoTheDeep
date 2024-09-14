package com.millburnx.utils

/**
 * Represents a point of intersection on a line
 */
open class Intersection<T>(val point: Vec2d, val line: T) {
    override fun toString(): String {
        return "Intersection(point=$point, line=$line)"
    }
}

class BezierIntersection(point: Vec2d, line: Bezier, val t: Double) : Intersection<Bezier>(point, line) {
    override fun toString(): String {
        return "BezierIntersection(point=$point, line=$line, t=$t)"
    }
}