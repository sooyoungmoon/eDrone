package org.etri.eDroneWizard;

import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.dialogs.TitleAreaDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;

public class Dialog_ParamIn_Text extends TitleAreaDialog {

	public Label label_type;
	public Label label_name;
	public Label label_3;
	public Label label_4;
	public Label label_5;
	public Label label_6;

	public Text text_value;
	public Label label_min;
	public Label label_max;
	public Label label_not;

	private Model_Base model;
	private Label label_description;
	private Button button_arg;

	public Dialog_ParamIn_Text(Shell parentShell, Model_Base model) {
		super(parentShell);
		this.model = model;

	}

	@Override
	public void create() {
		super.create();
		setTitle("파라미터 값 설정");
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
		label.setText("Description : ");
		label.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		gd.heightHint = 50;
		label_description = new Label(container, SWT.BORDER);
		label_description.setLayoutData(gd);

		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 2;
		gd.heightHint = 100;
		gd.grabExcessHorizontalSpace = true;

		GridLayout gll = new GridLayout(2, false);
		Group group = new Group(container, SWT.BORDER);
		group.setLayout(gll);
		group.setLayoutData(gd);
		group.setText("Condition");

		GridData gdd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_BEGINNING);

		Label lb = new Label(group, SWT.NULL);
		lb.setText("  min   :");
		label_min = new Label(group, SWT.NULL);
		label_min.setLayoutData(gdd);

		lb = new Label(group, SWT.NULL);
		lb.setText("  max   :");
		label_max = new Label(group, SWT.NULL);
		label_max.setLayoutData(gdd);

		lb = new Label(group, SWT.NULL);
		lb.setText("  not   :");
		label_not = new Label(group, SWT.NULL);
		label_not.setLayoutData(gdd);

		gd = new GridData();
		label = new Label(container, SWT.NULL);
		label.setText("   Value   :");
		label.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.heightHint = 30;
		text_value = new Text(container, SWT.BORDER);
		text_value.setLayoutData(gd);

		initialize();
		return area;
	}

	private void initialize() {

		if (model.isArg) {
			button_arg.setSelection(true);
		} else {
			button_arg.setSelection(false);
		}

		label_type.setText(":  " + model.type);
		label_name.setText(":  " + model.name);

		String value = model.value;

		if (value.indexOf(";") != -1) {
			String[] tokens = value.split(";");

			String[] tk2 = tokens[0].split(",");
			String min = tk2[0].equals("None") ? "" : tk2[0];
			String max = tk2[1].equals("None") ? "" : tk2[1];
			String not = tk2[2].equals("None") ? "" : tk2[2];

			label_min.setText(min);
			label_max.setText(max);
			label_not.setText(not);

			value = tokens[tokens.length - 1];
			if (value.equals("None")) {
				value = "";
			}
		}

		text_value.setText(value);
		label_description.setText(model.description);

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
		model.description = label_description.getText();
		String name = model.name;
		String min = label_min.getText().length() < 1 ? "None" : label_min.getText();
		String max = label_max.getText().length() < 1 ? "None" : label_max.getText();
		String not = label_not.getText().length() < 1 ? "None" : label_not.getText();
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

		super.okPressed();
	}

}