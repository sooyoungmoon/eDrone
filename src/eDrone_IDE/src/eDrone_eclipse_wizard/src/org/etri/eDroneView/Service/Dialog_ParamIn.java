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
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.TableItem;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_ComboItem;
import org.etri.eDroneModel.ParamType;

public class Dialog_ParamIn extends TitleAreaDialog {

	public Label label_type;
	public Label label_name;
	public Label label_3;
	public Label label_4;
	public Label label_5;
	public Label label_6;

	public Text text_value;
	public Text text_2;
	public Text text_3;
	public Text text_4;

	private Model_Base holder;

	private Button button_text;
	private Button button_combo;
	private Button button_create;
	private Button button_delete;
	private Model_Base model;
	private Label label_text;
	private Label label_combo;
	private Button button_rule;
	private TableViewer table_viewer;
	private Text text_description;
	private Button button_arg;

	public Dialog_ParamIn(Shell parentShell, Model_Base model) {
		super(parentShell);
		this.model = model;

	}

	@Override
	public void create() {
		super.create();
		setTitle("일반타입 파라미터 설정");
		setMessage("파라미터 입력을 텍스트 혹은 콤보박스 형태로 설정할 수 있습니다", IMessageProvider.INFORMATION);

	}

	@Override
	protected void createButtonsForButtonBar(final Composite parent) {

		super.createButton(parent, IDialogConstants.CANCEL_ID, IDialogConstants.CANCEL_LABEL, true);
		super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);
	}

	@Override
	protected Control createDialogArea(Composite parent) {

		getShell().setMinimumSize(400, 730);
		Composite area = (Composite) super.createDialogArea(parent);
		Composite container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(2, false);
		container.setLayout(layout);
		layout.marginTop = 20;
		layout.marginBottom = 10;
		layout.marginLeft = 20;
		layout.marginRight = 20;
		GridData gd = new GridData();
		Label label = new Label(container, SWT.NULL);
		label.setText("Argument?");
		label.setLayoutData(gd);

		button_arg = new Button(container, SWT.CHECK);
		gd = new GridData();
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

		Group group_input_type = new Group(container, SWT.BORDER);
		group_input_type.setText("Value");
		group_input_type.setLayoutData(gd);
		layout = new GridLayout();
		layout.numColumns = 3;
		group_input_type.setLayout(layout);

		gd = new GridData();
		button_text = new Button(group_input_type, SWT.RADIO);
		button_text.setLayoutData(gd);
		button_text.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				toggle_ui(false);
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_BEGINNING);

		label_text = new Label(group_input_type, SWT.NULL);
		label_text.setText("Text");
		label_text.setLayoutData(gd);

		gd = new GridData(GridData.HORIZONTAL_ALIGN_END);
		button_rule = new Button(group_input_type, SWT.PUSH);
		button_rule.setText("Value Condition");
		button_rule.setLayoutData(gd);

		button_rule.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				Dialog_Condition dc = new Dialog_Condition(getShell(), holder);
				dc.open();
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		gd = new GridData();
		Label dumLabel = new Label(group_input_type, SWT.NULL);
		dumLabel.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		text_value = new Text(group_input_type, SWT.BORDER);
		text_value.setLayoutData(gd);

		gd = new GridData();
		button_combo = new Button(group_input_type, SWT.RADIO);
		button_combo.setLayoutData(gd);
		button_combo.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				toggle_ui(true);

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_BEGINNING);
		label_combo = new Label(group_input_type, SWT.NULL);
		label_combo.setText("Combo Box");
		label_combo.setLayoutData(gd);

		Composite compoiste_buttons = new Composite(group_input_type, SWT.NULL);
		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_END | GridData.VERTICAL_ALIGN_CENTER);
		gd.heightHint = 40;
		compoiste_buttons.setLayoutData(gd);

		GridLayout gl = new GridLayout();
		gl.makeColumnsEqualWidth = true;
		gl.numColumns = 4;
		compoiste_buttons.setLayout(gl);

		button_create = new Button(compoiste_buttons, SWT.PUSH);
		button_create.setImage(Global.image_Add);
		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_END);
		gd.horizontalSpan = 3;
		button_create.setLayoutData(gd);
		button_create.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				Model_ComboItem mc = new Model_ComboItem("");

				Dialog_ComboItems dc = new Dialog_ComboItems(getShell(), mc);

				if (dc.open() == 0) {
					@SuppressWarnings("unchecked")
					ArrayList<Model_ComboItem> list = (ArrayList<Model_ComboItem>) table_viewer.getInput();

					list.add(mc);
					if (mc.isDefault) {
						for (Model_ComboItem m : list) {
							m.isDefault = false;
						}
						mc.isDefault = true;

					}

					table_viewer.refresh();
				}
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		button_delete = new Button(compoiste_buttons, SWT.PUSH);
		button_delete.setImage(Global.image_delete);
		gd = new GridData();
		button_delete.setLayoutData(gd);
		button_delete.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				IStructuredSelection is = table_viewer.getStructuredSelection();

				Model_ComboItem mc = (Model_ComboItem) is.getFirstElement();

				@SuppressWarnings("unchecked")
				ArrayList<Model_ComboItem> list = (ArrayList<Model_ComboItem>) table_viewer.getInput();
				list.remove(mc);
				table_viewer.refresh();

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		gd = new GridData();
		dumLabel = new Label(group_input_type, SWT.NULL);
		dumLabel.setLayoutData(gd);

		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 2;

		table_viewer = new TableViewer(group_input_type,
				SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		table_viewer.setContentProvider(new ContentProvider_combo_list());
		TableViewerColumn col = createTableViewerColumn(table_viewer, "Item Name", 300);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_ComboItem mc = (Model_ComboItem) element;
				return mc.name;
			}
		});

		table_viewer.addDoubleClickListener(new IDoubleClickListener() {

			@Override
			public void doubleClick(DoubleClickEvent event) {
				IStructuredSelection is = (IStructuredSelection) event.getSelection();
				Model_ComboItem mc = (Model_ComboItem) is.getFirstElement();

				Dialog_ComboItems dc = new Dialog_ComboItems(getShell(), mc);
				dc.open();

				if (mc.isDefault) {

					@SuppressWarnings("unchecked")
					ArrayList<Model_ComboItem> list = (ArrayList<Model_ComboItem>) table_viewer.getInput();

					for (Model_ComboItem m : list) {
						m.isDefault = false;
					}
					mc.isDefault = true;

				}

				table_viewer.refresh();
			}
		});
		TableViewerColumn col2 = createTableViewerColumn(table_viewer, "Default", 50);
		col2.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return "  ";
			}
		});

		Table table = table_viewer.getTable();
		table.setLinesVisible(true);
		table.setHeaderVisible(true);

		table.addListener(SWT.PaintItem, new Listener() {

			@Override
			public void handleEvent(Event event) {
				// Am I on collumn I need..?

				if (event.index == 1) {
					Image tmpImage = null;
					int tmpWidth = 0;
					int tmpHeight = 0;
					int tmpX = 0;
					int tmpY = 0;

					tmpWidth = table.getColumn(event.index).getWidth();
					tmpHeight = ((TableItem) event.item).getBounds().height;
					TableItem item = (TableItem) event.item;
					Model_ComboItem model = (Model_ComboItem) item.getData();
					if (model.isDefault) {
						tmpImage = Global.image_Accepted;
					} else {
						return;
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

		table_viewer.getTable().setLayoutData(gd);
		initialize();
		return area;
	}

	private class ContentProvider_combo_list implements IStructuredContentProvider {

		public Object[] getElements(Object input) {
			@SuppressWarnings("unchecked")
			ArrayList<Model_ComboItem> list = (ArrayList<Model_ComboItem>) input;
			return list.toArray();
		}

	}

	private void toggle_ui(boolean bool) {

		table_viewer.getTable().setEnabled(bool);
		button_create.setEnabled(bool);
		button_delete.setEnabled(bool);
		label_combo.setEnabled(bool);

		label_text.setEnabled(!bool);
		text_value.setEnabled(!bool);
		button_rule.setEnabled(!bool);
	}

	private void initialize() {

		holder = new Model_Base("holder", null);
		holder.type = model.type;

		String value = model.value;

		if (value.indexOf(";") != -1) {

			String[] tk = value.split(";");
			value = tk[tk.length - 1];

			String[] tk2 = tk[0].split(",");
			holder.min = tk2[0].equals("None") ? "" : tk2[0];
			holder.max = tk2[1].equals("None") ? "" : tk2[1];
			holder.not = tk2[2].equals("None") ? "" : tk2[2];

		}

		if (model.isArg) {
			button_arg.setSelection(true);
		} else {
			button_arg.setSelection(false);
		}

		label_type.setText(":  " + model.type);
		label_name.setText(":  " + model.name);
		text_description.setText(model.description);

		if (model.isComboBoxEditorNeeded || model.ptype.equals(ParamType.Combo)) {

			String tokens[] = model.back_value.split(",");
			ArrayList<Model_ComboItem> list = new ArrayList<Model_ComboItem>();
			if (tokens.length > 1) {

				for (int i = 1; i < tokens.length; i++) {

					Model_ComboItem mc = new Model_ComboItem(tokens[i]);

					list.add(mc);
					if (tokens[i].equals(model.value)) {
						mc.isDefault = true;
					}
				}

			}
			table_viewer.setInput(list);
			button_combo.setSelection(true);
			toggle_ui(true);

			table_viewer.refresh();
		} else {

			text_value.setText(value);
			table_viewer.setInput(new ArrayList<Model_ComboItem>());
			button_text.setSelection(true);
			toggle_ui(false);
			table_viewer.refresh();
		}

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

	// save content of the Text fields because they get disposed
	// as soon as the Dialog closes

	@Override
	protected void okPressed() {

		if (button_arg.getSelection()) {
			model.isArg = true;
		} else {
			model.isArg = false;
		}
		model.description = text_description.getText();
		if (button_text.getSelection()) {
			model.isComboBoxEditorNeeded = false;
			model.ptype = ParamType.Text;

			String name = model.name;
			String min = holder.min.length() < 1 ? "None" : holder.min;
			String max = holder.max.length() < 1 ? "None" : holder.max;
			String not = holder.not.length() < 1 ? "None" : holder.not;
			String value = text_value.getText().replace(" ", "");

			if (value.length() > 0) {

				String converted_type = Global.hashmap_type.get(model.type).beConverted;

				switch (converted_type) {
				case "int":
					Integer v = 0;
					try {
						v = Integer.parseInt(value);
					} catch (NumberFormatException e) {
						MessageDialog.openError(getShell(), "Warning", name + ": 값이 숫자가 아닙니다");
						return;
					}

					Integer compare = -1;

					if (min.equals("None") == false) {

						compare = Integer.parseInt(min);

						if (compare > v) {
							MessageDialog.openError(getShell(), "Warning", name + ": 최소값 (" + min + ") 미만입니다");
							return;
						}

					}
					if (max.equals("None") == false) {
						compare = Integer.parseInt(max);

						if (compare < v) {
							MessageDialog.openError(getShell(), "Warning", name + ": 최대값 (" + max + ") 초과입니다");
							return;
						}
					}
					if (not.equals("None") == false) {
						compare = Integer.parseInt(not);

						if (compare.equals(v)) {
							MessageDialog.openError(getShell(), "Warning", name + ": 입력 제한값 (" + not + ") 입니다");
							return;
						}
					}

					break;
				case "double":
					Double d = 0.0;
					try {
						d = Double.parseDouble(value);
					} catch (NumberFormatException e) {
						MessageDialog.openError(getShell(), "Warning", "값이 타입에 맞지 않습니다");
						return;
					}

					Double compared = 0.0;

					if (min.equals("None") == false) {

						compared = Double.parseDouble(min);

						if (compared > d) {
							MessageDialog.openError(getShell(), "Warning", name + ": 최소값 (" + min + ") 미만입니다");
							return;
						}

					}
					if (max.equals("None") == false) {
						compared = Double.parseDouble(max);

						if (compared < d) {
							MessageDialog.openError(getShell(), "Warning", name + ": 최대값 (" + max + ") 초과입니다");
							return;
						}
					}
					if (not.equals("None") == false) {
						compared = Double.parseDouble(not);

						if (compared.equals(d)) {
							MessageDialog.openError(getShell(), "Warning", name + ": 입력 제한값 (" + not + ") 입니다");
							return;
						}
					}

					break;
				default:

					if (not.equals("None") == false) {
						compared = Double.parseDouble(not);

						if (not.equals(value)) {
							MessageDialog.openError(getShell(), "Warning", name + ": 입력 제한값 (" + not + ") 입니다");
							return;
						}
					}
					break;
				}

			} else {
				value = "None";
			}

			model.value = min + "," + max + "," + not + ";" + value;

		} else {

			model.ptype = ParamType.Combo;
			model.isComboBoxEditorNeeded = true;
			@SuppressWarnings("unchecked")
			ArrayList<Model_ComboItem> list = (ArrayList<Model_ComboItem>) table_viewer.getInput();

			if (list.size() < 1) {
				MessageDialog.openError(getShell(), "Error", "콤보박스에 아이템을 추가해주세요");
				return;
			}

			StringBuilder sb = new StringBuilder("2");

			boolean isDefaultFound = false;

			for (Model_ComboItem mc : list) {

				if (mc.isDefault) {
					isDefaultFound = true;
					model.value = mc.name;
				}
				sb.append("," + mc.name);

			}
			if (isDefaultFound == false) {
				model.value = "";
			}

			model.back_value = sb.toString();

		}

		super.okPressed();
	}

}