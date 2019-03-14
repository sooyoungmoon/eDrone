package org.etri.eDroneView.Service;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Combo;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;

public class Dialog_Gen_Class extends Dialog {

	Model_Base model;
	Combo combo_type;
	Text text_name;

	Text text_value;
	private Text text_min;
	private Text text_max;
	private Text text_not;

	Label label_name;
	Label label_type;
	Label label_min;
	Label label_max;
	Label label_not;

	private boolean isEditable;

	public Dialog_Gen_Class(Shell parentShell, Model_Base m, boolean isEditable) {
		super(parentShell);
		this.model = m;
		this.isEditable = isEditable;
	}

	@Override
	public void create() {
		super.create();

	}

	@Override
	protected void configureShell(Shell newShell) {
		super.configureShell(newShell);
		newShell.setText("멤버 변수 생성/편집");
	}

	@Override
	protected void createButtonsForButtonBar(final Composite parent) {

		super.createButton(parent, IDialogConstants.CANCEL_ID, IDialogConstants.CANCEL_LABEL, true);
		super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);

	}

	@Override
	protected Control createDialogArea(Composite parent) {
		Composite area = (Composite) super.createDialogArea(parent);
		Composite container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(2, false);
		container.setLayout(layout);

		GridData gd = new GridData(GridData.FILL_HORIZONTAL);
		label_type = new Label(container, SWT.NULL);
		label_type.setLayoutData(gd);
		label_type.setText("Type: ");

		combo_type = new Combo(container, SWT.READ_ONLY);

		String items[] = new String[Global.hashmap_type.keySet().size()];

		int i = 0;
		for (String str : Global.hashmap_type.keySet()) {
			items[i] = str;
			i++;
		}

		combo_type.setItems(items);

		int j = 0;

		String type = model.type;

		for (String st : items) {
			if (st.equals(type)) {
				combo_type.select(j);
				break;
			}
			j++;
		}

		gd = new GridData(GridData.FILL_HORIZONTAL);
		label_name = new Label(container, SWT.NULL);
		label_name.setLayoutData(gd);
		label_name.setText("Name:");

		text_name = new Text(container, SWT.BORDER | SWT.SINGLE);
		text_name.setLayoutData(gd);
		text_name.setText(model.name);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		layout = new GridLayout(2, false);

		Group g_condition = new Group(container, SWT.BORDER);
		g_condition.setLayout(layout);
		g_condition.setLayoutData(gd);
		g_condition.setText("Condition");

		GridData normal_gd = new GridData();
		GridData hfill_gd = new GridData(GridData.FILL_HORIZONTAL);

		label_min = new Label(g_condition, SWT.NULL);
		label_min.setLayoutData(normal_gd);
		label_min.setText("min : ");

		text_min = new Text(g_condition, SWT.SINGLE | SWT.BORDER);
		text_min.setLayoutData(hfill_gd);

		label_max = new Label(g_condition, SWT.NULL);
		label_max.setLayoutData(normal_gd);
		label_max.setText("max : ");

		text_max = new Text(g_condition, SWT.SINGLE | SWT.BORDER);
		text_max.setLayoutData(hfill_gd);

		label_not = new Label(g_condition, SWT.NULL);
		label_not.setLayoutData(normal_gd);
		label_not.setText("not : ");

		text_not = new Text(g_condition, SWT.SINGLE | SWT.BORDER);
		text_not.setLayoutData(hfill_gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		Label label = new Label(container, SWT.NULL);
		label.setLayoutData(gd);
		label.setText("Value:");

		text_value = new Text(container, SWT.BORDER | SWT.SINGLE);
		text_value.setLayoutData(gd);

		checkType();
		fillText();

//		if (isEditable == false) {
//			g_condition.setEnabled(false);
//			label_name.setEnabled(false);
//			label_type.setEnabled(false);
//			text_name.setEnabled(false);
//			combo_type.setEnabled(false);
//			label_min.setEnabled(false);
//			label_max.setEnabled(false);
//			text_min.setEnabled(false);
//			text_max.setEnabled(false);
//			label_not.setEnabled(false);
//			text_not.setEnabled(false);
//
//		}

		return area;
	}

	private void checkType() {

		if (model.type.length() > 0) {
			String tobeType = Global.hashmap_type.get(model.type).beConverted;

			if (tobeType.equals("int") || tobeType.equals("double") || tobeType.equals("uint")) {

			} else {
				label_min.setEnabled(false);
				label_max.setEnabled(false);
				text_min.setEnabled(false);
				text_max.setEnabled(false);
			}
		}
	}

	private void fillText() {

		if (model.min.equals("None") == false) {
			text_min.setText(model.min);
		}
		if (model.max.equals("None") == false) {
			text_max.setText(model.max);
		}
		if (model.not.equals("None") == false) {
			text_not.setText(model.not);
		}
		if (model.value.equals("None") == false) {
			text_value.setText(model.value);
		}

	}

	@Override
	protected void okPressed() {

		String type = combo_type.getText().replace(" ", "");
		String name = text_name.getText().replace(" ", "");

		if (type.length() < 1) {
			MessageDialog.openError(getShell(), "Warning", "type을 입력해주세요");
			return;
		}
		if (name.length() < 1) {
			MessageDialog.openError(getShell(), "Warning", "name을 입력해주세요");
			return;
		}

		String min = text_min.getText().replace(" ", "").length() < 1 ? "None" : text_min.getText();
		String max = text_max.getText().replace(" ", "").length() < 1 ? "None" : text_max.getText();
		String not = text_not.getText().replace(" ", "").length() < 1 ? "None" : text_not.getText();
		String value = text_value.getText().replace(" ", "").length() < 1 ? "None" : text_value.getText();

		if (value.length() > 0) {

			String converted_type = Global.hashmap_type.get(type).beConverted;

			switch (converted_type) {
			case "int":
				Integer v = 0;

				try {
					if (value.equals("None") == false) {
						v = Integer.parseInt(value);
					}
					if (min.equals("None") == false) {
						Integer.parseInt(min);
					}
					if (max.equals("None") == false) {
						Integer.parseInt(max);
					}
					if (not.equals("None") == false) {
						Integer.parseInt(not);
					}
				} catch (NumberFormatException e) {
					MessageDialog.openError(getShell(), "Warning", "숫자가 아닌 입력이 존재합니다");
					return;
				}

				Integer compare = -1;

				if (min.equals("None") == false && value.equals("None") == false) {

					compare = Integer.parseInt(min);

					if (compare > v) {
						MessageDialog.openError(getShell(), "Warning", name + ": 최소값 (" + min + ") 미만입니다");
						return;
					}

				}
				if (max.equals("None") == false && value.equals("None") == false) {
					compare = Integer.parseInt(max);

					if (compare < v) {
						MessageDialog.openError(getShell(), "Warning", name + ": 최대값 (" + max + ") 초과입니다");
						return;
					}
				}
				if (not.equals("None") == false && value.equals("None") == false) {
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

					try {
						if (value.equals("None") == false) {
							d = Double.parseDouble(value);
						}
						if (min.equals("None") == false) {
							Double.parseDouble(min);
						}
						if (max.equals("None") == false) {
							Double.parseDouble(max);
						}
						if (not.equals("None") == false) {
							Double.parseDouble(not);
						}
					} catch (NumberFormatException e) {
						MessageDialog.openError(getShell(), "Warning", "숫자가 아닌 입력이 존재합니다");
						return;
					}
				} catch (NumberFormatException e) {
					MessageDialog.openError(getShell(), "Warning", "숫자가 아닌 입력이 존재합니다");
					return;
				}

				Double compared = 0.0;

				if (min.equals("None") == false && value.equals("None") == false) {

					compared = Double.parseDouble(min);

					if (compared > d) {
						MessageDialog.openError(getShell(), "Warning", name + ": 최소값 (" + min + ") 미만입니다");
						return;
					}

				}
				if (max.equals("None") == false && value.equals("None") == false) {
					compared = Double.parseDouble(max);

					if (compared < d) {
						MessageDialog.openError(getShell(), "Warning", name + ": 최대값 (" + max + ") 초과입니다");
						return;
					}
				}
				if (not.equals("None") == false && value.equals("None") == false) {
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

			String converted_type = Global.hashmap_type.get(type).beConverted;

			switch (converted_type) {
			case "int":
				try {
					if (min.equals("None") == false) {
						Integer.parseInt(min);
					}
					if (max.equals("None") == false) {
						Integer.parseInt(max);
					}
					if (not.equals("None") == false) {
						Integer.parseInt(not);
					}
				} catch (NumberFormatException e) {
					MessageDialog.openError(getShell(), "Warning", "숫자가 아닌 입력이 존재합니다");
					return;
				}
				break;
			case "double":
				try {
					if (min.equals("None") == false) {
						Double.parseDouble(min);
					}
					if (max.equals("None") == false) {
						Double.parseDouble(max);
					}
					if (not.equals("None") == false) {
						Double.parseDouble(not);
					}
				} catch (NumberFormatException e) {
					MessageDialog.openError(getShell(), "Warning", "숫자가 아닌 입력이 존재합니다");
					return;
				}
				break;
			}

		}
		model.min = min;
		model.max = max;
		model.not = not;
		model.type = type;
		model.name = name;
		model.value = value;
		super.okPressed();
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

}