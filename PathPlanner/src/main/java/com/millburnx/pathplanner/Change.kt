package com.millburnx.pathplanner

import com.millburnx.utils.BezierPoint
import com.millburnx.utils.Vec2d

interface Change {
    fun apply()
    fun undo()
}

class PointTranslation(val bezierPoint: BezierPoint, val type: BezierPoint.PointType, val change: Vec2d) : Change {
    // PointTranslation.apply will mostly be used for redoing the change
    // since creating a ton of new PointTranslations while dragging the point seems kind of inefficient
    override fun apply() {
        bezierPoint.updateType(type, bezierPoint.getType(type)!! + change)
    }

    override fun undo() {
        bezierPoint.updateType(type, bezierPoint.getType(type)!! - change)
    }

    override fun toString(): String {
        return "PointTranslation(type=$type, change=$change, bezierPoint=$bezierPoint)"
    }
}

class PointModification(
    val bezierPoint: BezierPoint,
    val modified: Boolean? = null,
    val mirrored: Boolean? = null,
    val split: Boolean? = null
) : Change {
    // Mostly for redoing the change, same/linked reasoning to PointTranslation
    override fun apply() {
        modified?.let { bezierPoint.modified = it }
        mirrored?.let { bezierPoint.mirrored = it }
        split?.let { bezierPoint.split = it }
    }

    override fun undo() {
        modified?.let { bezierPoint.modified = !it }
        mirrored?.let { bezierPoint.mirrored = !it }
        split?.let { bezierPoint.split = !it }
    }

    override fun toString(): String {
        return "PointModification(modified=$modified, mirrored=$mirrored, split=$split, bezierPoint=$bezierPoint)"
    }
}

class PointAddition(val pathPlanner: PathPlanner, val bezierPoint: BezierPoint, val index: Int) : Change {
    override fun apply() {
        pathPlanner.bezierPoints[pathPlanner.currentPath].add(bezierPoint)
    }

    override fun undo() {
        pathPlanner.removePointPure(bezierPoint)
    }

    override fun toString(): String {
        return "PointAddition(index=$index, bezierPoint=$bezierPoint)"
    }
}

class PointRemoval(val pathPlanner: PathPlanner, val bezierPoint: BezierPoint, val index: Int) : Change {
    val currentPath = pathPlanner.bezierPoints[pathPlanner.currentPath]
    val firstPrev = currentPath.getOrNull(1)?.prevHandle
    val lastNext = currentPath.dropLast(1).last().nextHandle
    val wasFirst = index == 0
    val wasLast = index == currentPath.size - 1

    override fun apply() {
        pathPlanner.removePointPure(bezierPoint)
    }

    override fun undo() {
        if (wasFirst) {
            currentPath.first().prevHandle = firstPrev
        }
        if (wasLast) {
            currentPath.last().nextHandle = lastNext
        }
        currentPath.add(index, bezierPoint)
    }

    override fun toString(): String {
        return "PointRemoval(index=$index, bezierPoint=$bezierPoint)"
    }
}