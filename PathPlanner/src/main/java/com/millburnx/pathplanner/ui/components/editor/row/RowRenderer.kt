package com.millburnx.pathplanner.ui.components.editor.row

import java.awt.Component
import javax.swing.JList
import javax.swing.ListCellRenderer

class RowRenderer : ListCellRenderer<Row> {
    override fun getListCellRendererComponent(
        list: JList<out Row>,
        value: Row,
        index: Int,
        isSelected: Boolean,
        cellHasFocus: Boolean,
    ): Component? {
        if (isSelected) {
            value.background = list.selectionBackground
            value.foreground = list.selectionForeground
        } else {
            value.index = index // set background color
        }
        return value
    }
}
