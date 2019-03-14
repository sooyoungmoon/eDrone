package org.etri.eDroneWizard;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.viewers.ArrayContentProvider;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.TableItem;
import org.etri.eDrone.Activator;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Base.Model_Condition;

public class Dialog_Condition extends Dialog {

	private TableViewer table_viewer = null;
	private Model_Base model;
	private static final Image image_Accepted = Activator.getImageDescriptor("icons/accept.png").createImage();
	private static final Image image_Denied = Activator.getImageDescriptor("icons/delete.png").createImage();

	public Dialog_Condition(Shell parentShell, Model_Base m) {
		super(parentShell);
		model = m;
	}

	@Override
	public void create() {
		super.create();
	}

	@Override
	protected void createButtonsForButtonBar(final Composite parent) {
		super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);
	}

	@Override
	protected Control createDialogArea(Composite parent) {
		Composite area = (Composite) super.createDialogArea(parent);
		Composite container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(1, false);
		container.setLayout(layout);

		table_viewer = new TableViewer(container,
				SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		TableViewerColumn col = createTableViewerColumn(table_viewer, "Condition", 200);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Condition mc = (Model_Condition) element;
				return mc.expression;
			}
		});

		col = createTableViewerColumn(table_viewer, "Status", 50);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return null;
			}
		});

		Table table = table_viewer.getTable();
		table.setLinesVisible(true);
		table.setHeaderVisible(true);

		table.addListener(SWT.PaintItem, new Listener() {

			@Override
			public void handleEvent(Event event) {

				if (event.index == 1) {
					Image tmpImage = null;
					int tmpWidth = 0;
					int tmpHeight = 0;
					int tmpX = 0;
					int tmpY = 0;

					tmpWidth = table.getColumn(event.index).getWidth();
					tmpHeight = ((TableItem) event.item).getBounds().height;
					TableItem item = (TableItem) event.item;
					Model_Condition model = (Model_Condition) item.getData();
					if (model.isSatisfied) {
						tmpImage = image_Accepted;
					} else {
						tmpImage = image_Denied;
					}

					tmpX = tmpImage.getBounds().width;
					tmpX = (tmpWidth / 2 - tmpX / 2);
					tmpY = tmpImage.getBounds().height;
					tmpY = (tmpHeight / 2 - tmpY / 2);
					if (tmpX <= 0)
						tmpX = event.x;
					else
						tmpX += event.x;
					if (tmpY <= 0)
						tmpY = event.y;
					else
						tmpY += event.y;
					event.gc.drawImage(tmpImage, tmpX, tmpY);

				}

				if (event.index == 4) {
					Image tmpImage = null;
					int tmpWidth = 0;
					int tmpHeight = 0;
					int tmpX = 0;
					int tmpY = 0;

					tmpWidth = table.getColumn(event.index).getWidth();
					tmpHeight = ((TableItem) event.item).getBounds().height;
					TableItem item = (TableItem) event.item;
					Model_Condition model = (Model_Condition) item.getData();
					if (model.isSatisfied) {
						tmpImage = image_Accepted;
					} else {
						tmpImage = image_Denied;
					}

					tmpX = tmpImage.getBounds().width;
					tmpX = (tmpWidth / 2 - tmpX / 2);
					tmpY = tmpImage.getBounds().height;
					tmpY = (tmpHeight / 2 - tmpY / 2);
					if (tmpX <= 0)
						tmpX = event.x;
					else
						tmpX += event.x;
					if (tmpY <= 0)
						tmpY = event.y;
					else
						tmpY += event.y;
					event.gc.drawImage(tmpImage, tmpX, tmpY);

				}
			}
		});

		GridData gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 7;
		gd.heightHint = 120;
		gd.grabExcessHorizontalSpace = true;
		table_viewer.getControl().setLayoutData(gd);
		table_viewer.setContentProvider(new ArrayContentProvider());

		if (model.hasCondition) {
			table_viewer.setInput(model.list_condition.toArray());
		}

		return area;
	}

	private TableViewerColumn createTableViewerColumn(TableViewer viewer, String title, int bound) {

		int align = SWT.CENTER;
		if (title.equals("Description")) {
			align = SWT.BEGINNING;
		}
		final TableViewerColumn viewerColumn = new TableViewerColumn(viewer, align);
		final TableColumn column = viewerColumn.getColumn();
		column.setText(title);
		column.setWidth(bound);
		column.setResizable(true);
		column.setMoveable(true);
		return viewerColumn;
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

}