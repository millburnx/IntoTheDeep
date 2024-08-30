package com.millburnx.pathplanner.ui.components.editor

import com.millburnx.pathplanner.ui.components.editor.row.Row
import com.millburnx.pathplanner.ui.components.editor.row.RowModel
import java.awt.datatransfer.DataFlavor
import java.awt.datatransfer.Transferable
import javax.swing.JComponent
import javax.swing.JList
import javax.swing.TransferHandler

class EditorTransferHandle(val list: JList<Row>) : TransferHandler() {
    override fun createTransferable(comp: JComponent): Transferable {
        val index = list.selectedIndex
        val panel = list.model.getElementAt(index)!!
        return object : Transferable {
            override fun getTransferData(flavor: DataFlavor): Any {
                require(flavor == Row.FLAVOR) { "Unsupported flavor: $flavor" }
                return panel
            }

            override fun getTransferDataFlavors(): Array<DataFlavor> {
                return arrayOf(Row.FLAVOR)
            }

            override fun isDataFlavorSupported(flavor: DataFlavor): Boolean {
                return flavor == Row.FLAVOR
            }
        }
    }

    override fun canImport(support: TransferSupport): Boolean {
        return support.isDataFlavorSupported(Row.FLAVOR)
    }

    override fun importData(support: TransferSupport): Boolean {
        if (!canImport(support)) {
            return false
        }

        val index = (support.dropLocation as JList.DropLocation).index

        if (index < 0 || index == list.selectedIndex) { // skip same index
            return false
        }

        val row = support.transferable.getTransferData(Row.FLAVOR) as Row
        val from = list.selectedIndex

        val model = list.model as RowModel
        model.removeRow(from)
        model.addRow(row, if (index > from) index - 1 else index)

        return true
    }

    override fun getSourceActions(comp: JComponent): Int {
        return MOVE
    }
}
