package org.etri.eDroneView.Service;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;

public class Dialog_Condition extends Dialog {

	@SuppressWarnings("unused")
	private Model_Base model;

	private Text text_min;
	private Text text_max;
	private Text text_not;

	public Dialog_Condition(Shell parentShell, Model_Base m) {
		super(parentShell);
		model = m;
	}

	@Override
	public void create() {
		super.create();
		getShell().setText("파라미터 조건 설정");
	}

	@Override
	protected void createButtonsForButtonBar(final Composite parent) {
		super.createButton(parent, IDialogConstants.CANCEL_ID, IDialogConstants.CANCEL_LABEL, true);
		super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);
	}

	@Override
	protected Control createDialogArea(Composite parent) {

		getShell().setMinimumSize(300, 200);
		Composite area = (Composite) super.createDialogArea(parent);
		Composite container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(2, false);
		container.setLayout(layout);

		GridData normal_gd = new GridData();
		GridData hfill_gd = new GridData(GridData.FILL_HORIZONTAL);

		Label label = new Label(container, SWT.NULL);
		GridData gd = new GridData();
		gd.widthHint = 50;
		label.setLayoutData(gd);
		label.setText("Type : ");

		label = new Label(container, SWT.NULL);
		label.setLayoutData(hfill_gd);
		if (Global.hashmap_type.containsKey(model.type))
			label.setText(model.type + " → " + Global.hashmap_type.get(model.type).beConverted);

		label = new Label(container, SWT.NULL);
		label.setLayoutData(normal_gd);
		label.setText("min : ");

		text_min = new Text(container, SWT.SINGLE | SWT.BORDER);
		text_min.setLayoutData(hfill_gd);

		label = new Label(container, SWT.NULL);
		label.setLayoutData(normal_gd);
		label.setText("max : ");

		text_max = new Text(container, SWT.SINGLE | SWT.BORDER);
		text_max.setLayoutData(hfill_gd);

		label = new Label(container, SWT.NULL);
		label.setLayoutData(normal_gd);
		label.setText("not : ");

		text_not = new Text(container, SWT.SINGLE | SWT.BORDER);
		text_not.setLayoutData(hfill_gd);

		checkType();
		fillText();
		return area;
	}

	private void checkType() {
		if (Global.hashmap_type.containsKey(model.type)) {
			String tobeType = Global.hashmap_type.get(model.type).beConverted;

			if (tobeType.equals("int") || tobeType.equals("double") || tobeType.equals("uint")) {

			} else {
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

	}

	@Override
	protected boolean isResizable() {
		return true;
	}

	@Override
	protected void okPressed() {
		String min = text_min.getText().length() < 1 ? "None" : text_min.getText();
		String max = text_max.getText().length() < 1 ? "None" : text_max.getText();
		String not = text_not.getText().length() < 1 ? "None" : text_not.getText();

		String converted_type = Global.hashmap_type.get(model.type).beConverted;

		switch (converted_type) {
		case "int":
			if (min.equals("None") == false) {
				try {
					Integer.parseInt(min);
				} catch (NumberFormatException ex) {
					MessageDialog.openError(getShell(), "Warning", "min 값이 숫자가 아닙니다");
					return;
				}

			}
			if (max.equals("None") == false) {
				try {
					Integer.parseInt(max);
				} catch (NumberFormatException ex) {
					MessageDialog.openError(getShell(), "Warning", "max 값이 숫자가 아닙니다");
					return;
				}
			}
			if (not.equals("None") == false) {
				try {
					Integer.parseInt(not);
				} catch (NumberFormatException ex) {
					MessageDialog.openError(getShell(), "Warning", "not 값이 숫자가 아닙니다");
					return;
				}
			}

		case "double":
			if (min.equals("None") == false) {
				try {
					Double.parseDouble(min);
				} catch (NumberFormatException ex) {
					MessageDialog.openError(getShell(), "Warning", "min 값이 숫자가 아닙니다");
					return;
				}

			}
			if (max.equals("None") == false) {
				try {
					Double.parseDouble(max);
				} catch (NumberFormatException ex) {
					MessageDialog.openError(getShell(), "Warning", "max 값이 숫자가 아닙니다");
					return;
				}
			}
			if (not.equals("None") == false) {
				try {
					Double.parseDouble(not);
				} catch (NumberFormatException ex) {
					MessageDialog.openError(getShell(), "Warning", "not 값이 숫자가 아닙니다");
					return;
				}
			}
		}
		model.min = min;
		model.max = max;
		model.not = not;

		super.okPressed();
	}

}