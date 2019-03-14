package org.etri.eDroneView.Service;

import java.util.ArrayList;

import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.IMessageProvider;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.dialogs.TitleAreaDialog;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;

public class Dialog_ParamIn_Class extends TitleAreaDialog {

	private Label label_type;
	private Label label_name;
	private Label label_combo;
	private Button button_create;
	private Button button_delete;
	private TableViewer table_viewer;
	private Text text_description;
	private Button button_arg;

	private Model_Base model;
	private boolean isEdtiable;

	public Dialog_ParamIn_Class(Shell parentShell, Model_Base model, boolean isEdtiable) {
		super(parentShell);
		this.model = model;
		this.isEdtiable = isEdtiable;
	}

	@Override
	public void create() {
		super.create();
		setTitle("기타 자료형(클래스) 파라미터 설정");
		setMessage("멤버변수 및 디폴트 값을 편집할 수 있습니다", IMessageProvider.INFORMATION);
	}

	@Override
	protected void createButtonsForButtonBar(final Composite parent) {

		super.createButton(parent, IDialogConstants.CANCEL_ID, IDialogConstants.CANCEL_LABEL, true);
		super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);
	}

	@Override
	protected Control createDialogArea(Composite parent) {

		getShell().setMinimumSize(300, 730);
		Composite area = (Composite) super.createDialogArea(parent);
		Composite container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(2, false);
		layout.marginTop = 20;
		layout.marginBottom = 10;
		layout.marginLeft = 20;
		layout.marginRight = 20;
		container.setLayout(layout);

		GridData gd = new GridData();
		Label label = new Label(container, SWT.NULL);
		label.setText("Argument?");
		label.setLayoutData(gd);

		button_arg = new Button(container, SWT.CHECK);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_arg.setLayoutData(gd);

		Label lbtFirstName = new Label(container, SWT.NONE);
		lbtFirstName.setText("Type");

		gd = new GridData();
		gd.grabExcessHorizontalSpace = true;
		gd.horizontalAlignment = GridData.FILL;

		label_type = new Label(container, SWT.NONE);
		label_type.setLayoutData(gd);

		Label lbtFirstName2 = new Label(container, SWT.NONE);
		lbtFirstName2.setText("Name");

		gd = new GridData();
		gd.grabExcessHorizontalSpace = true;
		gd.horizontalAlignment = GridData.FILL;

		label_name = new Label(container, SWT.NONE);
		label_name.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_BEGINNING);
		gd.horizontalSpan = 2;
		label = new Label(container, SWT.NULL);
		label.setText("Description");
		label.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		gd.heightHint = 100;
		text_description = new Text(container, SWT.MULTI | SWT.BORDER | SWT.WRAP);
		text_description.setLayoutData(gd);

		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 2;
		gd.heightHint = 120;
		gd.grabExcessHorizontalSpace = true;

		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_BEGINNING);
		label_combo = new Label(container, SWT.NULL);
		label_combo.setText("Class Definition");
		label_combo.setLayoutData(gd);

		Composite compoiste_buttons = new Composite(container, SWT.NULL);
		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_END | GridData.VERTICAL_ALIGN_CENTER);
		gd.heightHint = 40;
		compoiste_buttons.setLayoutData(gd);

		GridLayout gl = new GridLayout();
		gl.numColumns = 4;
		compoiste_buttons.setLayout(gl);

		button_create = new Button(compoiste_buttons, SWT.PUSH);
		button_create.setImage(Global.image_Add);
		button_create.setText("Add Item");
		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_END);
		gd.horizontalSpan = 3;
		button_create.setLayoutData(gd);
		button_create.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				Model_Base mb = new Model_Base("", "new", null);
				Dialog_Gen_Class dgc = new Dialog_Gen_Class(getShell(), mb, isEdtiable);

				if (dgc.open() == 0) {
					@SuppressWarnings("unchecked")
					ArrayList<Model_Base> list = (ArrayList<Model_Base>) table_viewer.getInput();
					list.add(mb);
					table_viewer.refresh();
				}
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		button_delete = new Button(compoiste_buttons, SWT.PUSH);
		button_delete.setImage(Global.image_delete);
		gd = new GridData(GridData.HORIZONTAL_ALIGN_END);
		button_delete.setText("Delete Item");
		button_delete.setLayoutData(gd);
		button_delete.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				IStructuredSelection is = table_viewer.getStructuredSelection();
				Model_Base mb = (Model_Base) is.getFirstElement();
				@SuppressWarnings("unchecked")
				ArrayList<Model_Base> list = (ArrayList<Model_Base>) table_viewer.getInput();
				list.remove(mb);
				table_viewer.refresh();
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 2;
		gd.heightHint = 300;
		table_viewer = new TableViewer(container,
				SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		table_viewer.setContentProvider(new ContentProvider_Class());
		table_viewer.getTable().setHeaderVisible(true);
		table_viewer.addDoubleClickListener(new IDoubleClickListener() {

			@Override
			public void doubleClick(DoubleClickEvent event) {
				IStructuredSelection is = (IStructuredSelection) event.getSelection();
				Model_Base mc = (Model_Base) is.getFirstElement();

				Dialog_Gen_Class dc = new Dialog_Gen_Class(getShell(), mc, isEdtiable);
				dc.open();

				table_viewer.refresh();
			}
		});

		table_viewer.getTable().setLayoutData(gd);

		initialize();
		return area;
	}

	private class ContentProvider_Class implements IStructuredContentProvider {

		public Object[] getElements(Object input) {
			@SuppressWarnings("unchecked")
			ArrayList<Model_Base> list = (ArrayList<Model_Base>) input;
			return list.toArray();
		}

	}

	private void initialize() {

		label_type.setText(":  " + model.type);
		label_name.setText(":  " + model.name);
		text_description.setText(model.description);

		TableViewerColumn tv_type = createTableViewerColumn(table_viewer, "Type", 200);
		TableViewerColumn tv_name = createTableViewerColumn(table_viewer, "Name", 200);
		TableViewerColumn tv_value = createTableViewerColumn(table_viewer, "Value", 200);

		tv_type.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Base mc = (Model_Base) element;
				return mc.type;
			}
		});

		tv_name.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Base mc = (Model_Base) element;
				return mc.name;
			}
		});

		tv_value.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Base mc = (Model_Base) element;
				return mc.value.equals("None") ? "" : mc.value;
			}
		});

		String payload = model.value;
		String tokens[] = payload.split(";");

		ArrayList<Model_Base> list = new ArrayList<Model_Base>();

		if (tokens[0].length() > 0) {

			Integer num_columns = Integer.parseInt(tokens[0]);
			int i = 1;
			for (; i < num_columns + 1; i++) {
				String token = tokens[i];
				String mini_tokens[] = token.split(",");

				Model_Base mb = new Model_Base(mini_tokens[0], mini_tokens[1], null);
				mb.value = mini_tokens[2].equals("None") ? "" : mini_tokens[2];
				mb.min = mini_tokens[3].equals("None") ? "" : mini_tokens[3];
				mb.max = mini_tokens[4].equals("None") ? "" : mini_tokens[4];
				mb.not = mini_tokens[5].equals("None") ? "" : mini_tokens[5];
				list.add(mb);
			}

		}

		table_viewer.setInput(list);

	}

	private TableViewerColumn createTableViewerColumn(TableViewer viewer, String title, int bound) {

		int align = SWT.CENTER;
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

	@Override
	protected void okPressed() {
		if (button_arg.getSelection()) {
			model.isArg = true;
		} else {
			model.isArg = false;
		}
		model.description = text_description.getText();

		StringBuilder sb = new StringBuilder();
		@SuppressWarnings("unchecked")
		ArrayList<Model_Base> list = (ArrayList<Model_Base>) table_viewer.getInput();
		if (list.size() < 1) {
			MessageDialog.openError(getShell(), "Warning", "최소 하나 필드가 있어야 합니다");
			return;
		}

		sb.append(Integer.toString(list.size()));
		sb.append(";");

		for (Model_Base mb : list) {

			String type = mb.type.replace(" ", "").length() < 1 ? "None" : mb.type;
			String name = mb.name.replace(" ", "").length() < 1 ? "None" : mb.name;
			String value = mb.value.replace(" ", "").length() < 1 ? "None" : mb.value;
			String min = mb.min.replace(" ", "").length() < 1 ? "None" : mb.min;
			String max = mb.max.replace(" ", "").length() < 1 ? "None" : mb.max;
			String not = mb.not.replace(" ", "").length() < 1 ? "None" : mb.not;
			sb.append(type + "," + name + "," + value + "," + min + "," + max + "," + not);
			sb.append(";");
		}

		model.value = sb.toString();

		super.okPressed();
	}

}