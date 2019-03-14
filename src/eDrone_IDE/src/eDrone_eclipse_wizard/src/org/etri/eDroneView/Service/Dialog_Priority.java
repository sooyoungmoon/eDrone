package org.etri.eDroneView.Service;

import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.IMessageProvider;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.dialogs.TitleAreaDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.etri.eDroneModel.Model_Base;

public class Dialog_Priority extends TitleAreaDialog {

	public Label label_1;
	public Label label_2;
	public Label label_3;
	public Label label_4;
	public Label label_5;
	public Label label_6;

	public Text txtPriority;

	public Button chkIsDefault;

	Model_Base model;

	public Dialog_Priority(Shell parentShell, Model_Base model) {
		super(parentShell);
		this.model = model;

	}

	@Override
	public void create() {
		super.create();
		setTitle("API 우선순위 설정");
		setMessage("API 우선순위 값을 설정하세요", IMessageProvider.INFORMATION);
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

		createLabel(container, "API Name", label_1);
		createLabel(container, "Project", label_2);
		createText(container, "Priority", txtPriority);

		Label lbtFirstName = new Label(container, SWT.NONE);
		lbtFirstName.setText("API Name");

		GridData gd = new GridData();
		gd.grabExcessHorizontalSpace = true;
		gd.horizontalAlignment = GridData.FILL;

		label_1 = new Label(container, SWT.NONE);
		label_1.setLayoutData(gd);

		Label lbtFirstName2 = new Label(container, SWT.NONE);
		lbtFirstName2.setText("Project");

		gd = new GridData();
		gd.grabExcessHorizontalSpace = true;
		gd.horizontalAlignment = GridData.FILL;

		label_2 = new Label(container, SWT.NONE);
		label_2.setLayoutData(gd);

		Label lbtFirstName3 = new Label(container, SWT.NONE);
		lbtFirstName3.setText("Priority");

		gd = new GridData();
		gd.grabExcessHorizontalSpace = true;
		gd.horizontalAlignment = GridData.FILL;
		txtPriority = new Text(container, SWT.BORDER);
		txtPriority.setLayoutData(gd);

		label_1.setText(model.name);
		label_2.setText(model.getProjectParent().name);
		txtPriority.setText(Double.toString(model.priority));

		return area;
	}

	private void createLabel(Composite container, String name, Label label) {

	}

	private void createText(Composite container, String name, Text text) {

	}

	@Override
	protected boolean isResizable() {
		return true;
	}

	@Override
	protected void okPressed() {

		try {
			model.priority = Double.parseDouble(txtPriority.getText());

		} catch (Exception e) {

			MessageDialog.openError(getShell(), "Error", "숫자를 입력해주세요");
			return;
		}

		super.okPressed();
	}

}