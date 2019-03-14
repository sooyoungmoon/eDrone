package org.etri.eDroneLauncher;

import java.io.File;
import java.io.FileFilter;

import org.eclipse.core.runtime.CoreException;
import org.eclipse.debug.core.ILaunchConfiguration;
import org.eclipse.debug.core.ILaunchConfigurationWorkingCopy;
import org.eclipse.debug.ui.AbstractLaunchConfigurationTab;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.ModifyEvent;
import org.eclipse.swt.events.ModifyListener;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Combo;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;

public class LaunchTab extends AbstractLaunchConfigurationTab {

	public static Combo combo_device_choice;
	public static Text text_address;
	public static Text text_port;
	public static Text text_username;
	public static Text text_password;
	public static Text text_catkinws_path;
	public static Combo combo_projectName;
	public static Text text_projectLoc;
	public static Text text_arguments;
	public static Button checkbox_isLog;
	public static Button checkbox_isRunOnly;
	public static Button checkbox_isCrossCompile;
	private Label label_targetdevice;
	private Label label_targetaddress;
	private Label label_targetport;
	private Label label_targetuser;
	private Label label_targetpassword;
	private Label label_catkin_wspath;

	@Override
	public void createControl(Composite parent) {

		Composite comp = new Group(parent, SWT.BORDER);
		setControl(comp);

		GridLayout gl = new GridLayout();
		gl.marginLeft = 15;
		gl.marginRight = 15;
		gl.marginTop = 5;
		gl.marginBottom = 5;
		comp.setLayout(gl);

		// Group - Target Info.

		Group GTarget = new Group(comp, SWT.NULL);
		GridData gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalAlignment = SWT.FILL;
		gd.grabExcessHorizontalSpace = true;
		GTarget.setLayoutData(gd);
		GTarget.setText("Target Info.");
		gl = new GridLayout();
		gl.numColumns = 10;
		gl.marginLeft = 15;
		gl.marginRight = 15;
		gl.marginTop = 5;
		gl.marginBottom = 5;
		GTarget.setLayout(gl);

		label_targetdevice = new Label(GTarget, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 2;
		label_targetdevice.setLayoutData(gd);
		label_targetdevice.setText("&Target Device :");

		combo_device_choice = new Combo(GTarget, SWT.NONE);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 6;
		combo_device_choice.setLayoutData(gd);

		String[] arr = { "x86_64", "odroid" };
		combo_device_choice.setItems(arr);
		combo_device_choice.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				switch (combo_device_choice.getText()) {

				case "x86_64":

					toggleEnable(false);

					break;
				case "odroid":
					toggleEnable(true);
					break;
				default:
					toggleEnable(false);
					break;

				}

				updateLaunchConfigurationDialog();

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
				switch (combo_device_choice.getText()) {

				case "x86_64":

					toggleEnable(false);

					break;
				case "odroid":
					toggleEnable(true);
					break;
				default:
					toggleEnable(false);
					break;

				}

			}
		});

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		checkbox_isCrossCompile = new Button(GTarget, SWT.CHECK);
		checkbox_isCrossCompile.setText("Cross Compile");
		checkbox_isCrossCompile.setLayoutData(gd);
		checkbox_isCrossCompile.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				if (checkbox_isCrossCompile.getSelection()) {
					String location = System.getenv("CROSS_CATKIN_WS_PATH");
					text_projectLoc.setText(location);
					text_catkinws_path.setText("/home/" + combo_device_choice.getText() + "/catkin_ws");
				} else {
					String location = System.getenv("CATKIN_WS_PATH");
					text_projectLoc.setText(location);
					text_catkinws_path.setText(location);
				}
				updateLaunchConfigurationDialog();

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		label_targetaddress = new Label(GTarget, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 2;
		label_targetaddress.setLayoutData(gd);
		label_targetaddress.setText("&Target Address :");

		text_address = new Text(GTarget, SWT.BORDER | SWT.SINGLE);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 4;
		text_address.setLayoutData(gd);
		text_address.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				updateLaunchConfigurationDialog();
			}
		});

		label_targetport = new Label(GTarget, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 1;
		label_targetport.setLayoutData(gd);
		label_targetport.setText("&Port :");

		text_port = new Text(GTarget, SWT.BORDER | SWT.SINGLE);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 3;
		text_port.setLayoutData(gd);
		text_port.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				updateLaunchConfigurationDialog();
			}
		});

		label_targetuser = new Label(GTarget, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 2;
		label_targetuser.setLayoutData(gd);
		label_targetuser.setText("&User :");

		text_username = new Text(GTarget, SWT.BORDER | SWT.SINGLE);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 4;
		text_username.setLayoutData(gd);
		text_username.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				updateLaunchConfigurationDialog();
			}
		});

		label_targetpassword = new Label(GTarget, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 1;
		label_targetpassword.setLayoutData(gd);
		label_targetpassword.setText("&Password :");

		text_password = new Text(GTarget, SWT.BORDER | SWT.PASSWORD);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 3;
		text_password.setLayoutData(gd);
		text_password.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				updateLaunchConfigurationDialog();
			}
		});

		label_catkin_wspath = new Label(GTarget, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 2;
		label_catkin_wspath.setLayoutData(gd);
		label_catkin_wspath.setText("&Catkin Workspace Path :");

		text_catkinws_path = new Text(GTarget, SWT.BORDER | SWT.SINGLE);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 8;
		text_catkinws_path.setLayoutData(gd);
		text_catkinws_path.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				updateLaunchConfigurationDialog();
			}
		});

		// Group - Project Info.

		Group GProject = new Group(comp, SWT.NULL);
		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalAlignment = SWT.FILL;
		gd.grabExcessHorizontalSpace = true;
		GProject.setLayoutData(gd);
		GProject.setText("Project Info.");
		gl = new GridLayout();
		gl.numColumns = 10;
		gl.marginLeft = 15;
		gl.marginRight = 15;
		gl.marginTop = 5;
		gl.marginBottom = 5;
		GProject.setLayout(gl);

		Label label = new Label(GProject, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 2;
		label.setLayoutData(gd);
		label.setText("&Project Name :");

		combo_projectName = new Combo(GProject, SWT.BORDER);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 8;
		combo_projectName.setLayoutData(gd);
		combo_projectName.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				updateLaunchConfigurationDialog();
			}
		});

		File ws_file = new File(Global.CATKIN_WORKSPACE_PATH + "/src/");

		FileFilter filter2 = new FileFilter() {

			@Override
			public boolean accept(File file) {
				if (file.isDirectory()) {
					for (File f : file.listFiles()) {
						if (f.getName().equals("package.xml")) {
							return true;
						}
					}
				}
				return false;
			}
		};

		File[] projs = ws_file.listFiles(filter2);

		String[] names = new String[projs.length];

		int i = 0;
		for (File f : projs) {
			names[i] = f.getName();
			i++;
		}

		combo_projectName.setItems(names);

		label = new Label(GProject, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 2;
		label.setLayoutData(gd);
		label.setText("&Catkin Workspace Path :");

		text_projectLoc = new Text(GProject, SWT.BORDER | SWT.SINGLE);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 8;
		text_projectLoc.setLayoutData(gd);

		text_projectLoc.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				updateLaunchConfigurationDialog();
			}
		});

		label = new Label(GProject, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 2;
		label.setLayoutData(gd);
		label.setText("&Arguments :");

		text_arguments = new Text(GProject, SWT.BORDER | SWT.SINGLE);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 8;
		text_arguments.setLayoutData(gd);

		text_arguments.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				updateLaunchConfigurationDialog();
			}
		});

		Group GOptions = new Group(comp, SWT.NULL);
		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalAlignment = SWT.FILL;
		gd.grabExcessHorizontalSpace = true;
		GOptions.setLayoutData(gd);
		GOptions.setText("Options");
		gl = new GridLayout();
		gl.numColumns = 10;
		gl.marginLeft = 15;
		gl.marginRight = 15;
		gl.marginTop = 5;
		gl.marginBottom = 5;
		GOptions.setLayout(gl);

		checkbox_isLog = new Button(GOptions, SWT.CHECK);
		checkbox_isLog.setText("Show Launch Logs");
		checkbox_isLog.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				updateLaunchConfigurationDialog();

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		checkbox_isRunOnly = new Button(GOptions, SWT.CHECK);
		checkbox_isRunOnly.setText("Run Only");
		checkbox_isRunOnly.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				updateLaunchConfigurationDialog();

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

	}

	private void toggleEnable(boolean bol) {

		label_catkin_wspath.setEnabled(bol);
		label_targetaddress.setEnabled(bol);
		label_targetport.setEnabled(bol);

		text_catkinws_path.setEnabled(bol);
		text_address.setEnabled(bol);
		text_port.setEnabled(bol);
		checkbox_isCrossCompile.setEnabled(bol);

		if (bol == false) {
			checkbox_isCrossCompile.setSelection(false);
			text_address.setText("localhost");
			text_port.setText("22");
//			text_projectLoc.setText(System.getenv("CATKIN_WS_PATH"));
		}

	}

	@Override
	public void setDefaults(ILaunchConfigurationWorkingCopy configuration) {
	}

	@Override
	public void initializeFrom(ILaunchConfiguration configuration) {
		try {
			String device = configuration.getAttribute(LaunchConfigurationAttributes.TARGET_DEVICE, "odroid");
			String address = configuration.getAttribute(LaunchConfigurationAttributes.TARGET_ADDRESS, "localhost");
			String port = configuration.getAttribute(LaunchConfigurationAttributes.TARGET_PORT, "22");
			String user = configuration.getAttribute(LaunchConfigurationAttributes.USER, "");
			String password = configuration.getAttribute(LaunchConfigurationAttributes.PASSWORD, "");
			String path = configuration.getAttribute(LaunchConfigurationAttributes.CATKIN_WS_PATH,
					"/home/" + device + "/catkin_ws");
			String name = configuration.getAttribute(LaunchConfigurationAttributes.PROJECT_NAME, "");
			String arguments = configuration.getAttribute(LaunchConfigurationAttributes.ARGUMENTS, "");
			String isLog = configuration.getAttribute(LaunchConfigurationAttributes.ISLOG, "NO");
			String isRunOnly = configuration.getAttribute(LaunchConfigurationAttributes.ISRUNONLY, "NO");
			String isCross = configuration.getAttribute(LaunchConfigurationAttributes.ISCROSS, "NO");

			boolean isFound = false;
			for (int i = 0; i < combo_device_choice.getItemCount(); i++) {
				if (combo_device_choice.getItem(i).toLowerCase().equals(device.toLowerCase())) {
					isFound = true;

					combo_device_choice.select(i);
				}
			}

			if (isFound == false) {
				combo_device_choice.add(device);
				combo_device_choice.select(0);
			}

			text_address.setText(address);
			text_port.setText(port);
			text_username.setText(user);
			text_password.setText(password);
			text_catkinws_path.setText(path);

			int i = 0;
			for (String item : combo_projectName.getItems()) {
				if (item.equals(name)) {
					combo_projectName.select(i);
				}
				i++;
			}

			text_arguments.setText(arguments);

			if (isLog.equals("YES")) {
				checkbox_isLog.setSelection(true);
			}

			if (isRunOnly.equals("YES")) {
				checkbox_isRunOnly.setSelection(true);
			}

			if (isCross.equals("YES")) {
				checkbox_isCrossCompile.setSelection(true);
				String location = System.getenv("CROSS_CATKIN_WS_PATH");
				if (location != null)
					text_projectLoc.setText(location);

			} else {
				checkbox_isCrossCompile.setSelection(false);
				String location = System.getenv("CATKIN_WS_PATH");
				if (location != null)
					text_projectLoc.setText(location);
			}

			switch (combo_device_choice.getText()) {
			case "x86_64":
				toggleEnable(false);
				break;
			case "odroid":
				toggleEnable(true);
				break;
			default:
				toggleEnable(false);
				break;
			}

			updateLaunchConfigurationDialog();

		} catch (CoreException e) {
		}

		setDirty(false);
	}

	@Override
	public void performApply(ILaunchConfigurationWorkingCopy configuration) {
		configuration.setAttribute(LaunchConfigurationAttributes.TARGET_DEVICE, combo_device_choice.getText());
		configuration.setAttribute(LaunchConfigurationAttributes.TARGET_ADDRESS, text_address.getText());
		configuration.setAttribute(LaunchConfigurationAttributes.TARGET_PORT, text_port.getText());
		configuration.setAttribute(LaunchConfigurationAttributes.USER, text_username.getText());
		configuration.setAttribute(LaunchConfigurationAttributes.PASSWORD, text_password.getText());
		configuration.setAttribute(LaunchConfigurationAttributes.CATKIN_WS_PATH, text_catkinws_path.getText());
		configuration.setAttribute(LaunchConfigurationAttributes.PROJECT_NAME, combo_projectName.getText());
		configuration.setAttribute(LaunchConfigurationAttributes.PROJECT_LOCATION, text_projectLoc.getText());
		configuration.setAttribute(LaunchConfigurationAttributes.ARGUMENTS, text_arguments.getText());
		if (checkbox_isLog.getSelection() == true) {
			configuration.setAttribute(LaunchConfigurationAttributes.ISLOG, "YES");
		} else {
			configuration.setAttribute(LaunchConfigurationAttributes.ISLOG, "NO");
		}

		if (checkbox_isRunOnly.getSelection() == true) {
			configuration.setAttribute(LaunchConfigurationAttributes.ISRUNONLY, "YES");
		} else {
			configuration.setAttribute(LaunchConfigurationAttributes.ISRUNONLY, "NO");
		}

		if (checkbox_isCrossCompile.getSelection() == true) {
			configuration.setAttribute(LaunchConfigurationAttributes.ISCROSS, "YES");
		} else {
			configuration.setAttribute(LaunchConfigurationAttributes.ISCROSS, "NO");
		}
	}

	@Override
	public String getName() {
		return "eDrone launch tab";
	}

	@Override
	public Image getImage() {
		return Global.image_Logo;
	}

}
