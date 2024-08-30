package com.millburnx.pathplanner.ui.components.editor

import com.millburnx.pathplanner.ui.components.editor.row.Row
import com.millburnx.pathplanner.ui.components.editor.row.RowModel
import com.millburnx.pathplanner.ui.components.editor.row.RowRenderer
import javax.swing.DropMode
import javax.swing.JList
import javax.swing.ListSelectionModel

class EditorList(val model: RowModel = RowModel()) : JList<Row>(model) {
    init {
        dropMode = DropMode.INSERT
        dragEnabled = true
        selectionMode = ListSelectionModel.SINGLE_SELECTION
        cellRenderer = RowRenderer()
        transferHandler = EditorTransferHandle(this)
    }
}