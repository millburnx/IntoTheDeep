package com.millburnx.pathPlanner.layout

import com.millburnx.pathPlanner.components.View
import com.millburnx.util.geometry.Vec2d
import javax.swing.JPanel
import kotlin.math.min
import kotlin.math.roundToInt

public class ViewWrapper : JPanel() {
    public val padding: Vec2d = Vec2d(24, 0)

    public val content: View = View()

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