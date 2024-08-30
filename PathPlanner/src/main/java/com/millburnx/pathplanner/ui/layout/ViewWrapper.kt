package com.millburnx.pathplanner.ui.layout

import com.millburnx.pathplanner.ui.components.View
import com.millburnx.utils.vec2d.Vec2d
import javax.swing.JPanel
import kotlin.math.min
import kotlin.math.roundToInt

class ViewWrapper : JPanel() {
    val padding = Vec2d(24, 0)

    val content = View()

    init {
        layout = null // handle layout manually
        add(content)
    }

    override fun doLayout() {
        super.doLayout()
        val size = Vec2d(width, height) - padding * 2
        val min = min(size.x, size.y)
        val contentSize = Vec2d(min)

        val offsets = (size - contentSize) / 2 + padding

        content.setBounds(
            offsets.x.roundToInt(),
            offsets.y.roundToInt(),
            contentSize.x.roundToInt(),
            contentSize.y.roundToInt()
        )
    }
}