package com.millburnx.pathplanner.ui.components.editor.row

import com.millburnx.pathplanner.ui.Theme
import com.millburnx.utils.vec2d.Vec2dInt
import java.awt.GridBagConstraints
import java.awt.GridBagLayout
import java.awt.datatransfer.DataFlavor
import java.io.Serializable
import javax.swing.JComponent
import javax.swing.JPanel

class Row(val children: List<JComponent>) : JPanel(), Serializable {
    var index = -1
        set(value) {
            background = if (value % 2 == 0) Theme.bg1 else Theme.bg0
        }

    companion object {
        val FLAVOR = DataFlavor(JPanel::class.java, "JPanel")
    }

    constructor(vararg children: JComponent) : this(children.toList())

    val padding = Vec2dInt(12, 8)

    init {
        layout = GridBagLayout()
        val constraints = GridBagConstraints()
        constraints.gridy = 0
        constraints.insets.set(padding.y, padding.x, padding.y, padding.x)
        children.forEachIndexed { index, component ->
            constraints.gridx = index
            add(component, constraints)
        }
    }

    fun updateCellMargins(maxWidths: List<Int>) {
        children.forEachIndexed { index, component ->
            val width = component.preferredSize.width
            val max = maxWidths[index]
            val diff = max - width
            val gridBag = layout as GridBagLayout
            val constraints = GridBagConstraints()
            constraints.gridx = index
            constraints.insets.set(padding.y, padding.x, padding.y, padding.x)
            constraints.insets.right = diff.coerceAtLeast(0) + padding.x
            gridBag.setConstraints(component, constraints)
            println("Width: $width, Max: $max, Diff: $diff")
        }
    }
}
