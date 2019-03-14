package org.etri.eDroneView.Service;

import org.eclipse.jface.viewers.ArrayContentProvider;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.FocusEvent;
import org.eclipse.swt.events.FocusListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;

public class Table_ParamOut {

	private TableViewer viewer;
	private Composite parent;

	private TableViewerColumn createTableViewerColumn(String title, int bound, final int colNumber) {

		int align = SWT.CENTER;

		final TableViewerColumn viewerColumn = new TableViewerColumn(viewer, align);
		final TableColumn column = viewerColumn.getColumn();
		column.setText(title);
		column.setWidth(bound);
		column.setResizable(true);
		column.setMoveable(true);
		return viewerColumn;
	}

	private void createColumns2(final Composite parent, final TableViewer viewer) {

		String[] titles = { "Type", "Name", "Description" };
		int[] bounds = { 200, 200, 350 };

		TableViewerColumn col = createTableViewerColumn(titles[0], bounds[0], 0);
		col.setLabelProvider(new ColumnLabelProvider() {

			@Override
			public String getText(Object element) {
				return ((Model_Base) element).type;
			}
		});

		col = createTableViewerColumn(titles[1], bounds[1], 1);
		col.setLabelProvider(new ColumnLabelProvider() {

			@Override
			public String getText(Object element) {
				return ((Model_Base) element).name;
			}
		});

		col = createTableViewerColumn(titles[2], bounds[2], 2);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Base model = (Model_Base) element;
				return model.description;
			}
		});

	}

	public Table_ParamOut(Composite c) {

		parent = c;
		viewer = new TableViewer(c, SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);
		createColumns2(c, viewer);

		final Table table = viewer.getTable();

		table.setLinesVisible(true);
		table.setHeaderVisible(true);

		GridData gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 7;
		gd.heightHint = 80;
		gd.grabExcessHorizontalSpace = true;
		viewer.getControl().setLayoutData(gd);
		viewer.setContentProvider(new ArrayContentProvider());

		Global.dialog_opened.tableviewer_paramout = viewer;

		viewer.addDoubleClickListener(new IDoubleClickListener() {
			@Override
			public void doubleClick(DoubleClickEvent event) {
				IStructuredSelection is = (IStructuredSelection) event.getSelection();
				Model_Base model = (Model_Base) is.getFirstElement();

				if (model == null)
					return;
				System.out.println(model.name);
				Dialog_ParamOut dp = new Dialog_ParamOut(parent.getShell(), model);
				dp.open();
				viewer.refresh();
			}

		});
		viewer.getTable().addFocusListener(new FocusListener() {

			@Override
			public void focusLost(FocusEvent e) {
				viewer.getTable().deselectAll();
			}

			@Override
			public void focusGained(FocusEvent e) {

			}
		});

	}

}
