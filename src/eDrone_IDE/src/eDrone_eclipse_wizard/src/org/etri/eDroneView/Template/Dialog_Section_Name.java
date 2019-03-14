package org.etri.eDroneView.Template;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.etri.eDroneModel.Model_Section;

public class Dialog_Section_Name extends Dialog {

	Model_Section model;
	Text text;

	public Dialog_Section_Name(Shell parentShell, Model_Section m) {
		super(parentShell);
		this.model = m;
	}

	@Override
	public void create() {
		super.create();
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

		GridData gd = new GridData();
		Label label = new Label(container, SWT.NULL);
		label.setText("Name : ");
		label.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		text = new Text(container, SWT.BORDER);
		text.setLayoutData(gd);
		text.setText(model.name);

		if (model.name.equals("section_main") || model.name.equals("section_end")) {
			text.setText(model.name + " (기본 섹션 수정 불가)");
			text.setEnabled(false);
		}
		return area;
	}

	@Override
	protected void okPressed() {
		if (model.name.equals("section_main") || model.name.equals("section_end")) {
			super.okPressed();
			return;
		}
		model.name = text.getText();
		super.okPressed();
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

}