package com.millburnx.util.geometry

import com.millburnx.util.geometry.basic.vector.Vec2d
import java.io.Serializable

// make parent a pair ie Pair<Parent, Double> if you need more data such as T value
public data class Intersection<Parent>(val parent: Parent, val point: Vec2d) : Serializable