package org.etri.eDroneView.Service;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.IMessageProvider;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.dialogs.TitleAreaDialog;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.FocusEvent;
import org.eclipse.swt.events.FocusListener;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.DirectoryDialog;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.TabFolder;
import org.eclipse.swt.widgets.TabItem;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Service;
import org.etri.eDroneModel.ParamType;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

public class Dialog_Service_Edit extends TitleAreaDialog {

	public static String projectName;

	// 두번째 페이지 가용 API 트리의 static 변수
	public TreeViewer treeviewer_availabe_api;
	// 두번 페이지 선택된 API 테이블의 static 변수
	public TableViewer tableviewer_selected_api;
	// 두번째 페이지 선택된 API 중 클릭된 API의 입력 파라미터를 보여주는 테이블의 static 변수
	public TableViewer tableviewer_paramin;
	// 두번째 페이지 선택된 API 중 클릭된 API의 출력 파라미터를 보여주는 테이블의 static 변수
	public TableViewer tableviewer_paramout;
	// 두번째 페이지 상단 두번째 텝에 선택된 API들의 의존 프로젝트를 보여주는 테이블의 static 변수
	public TableViewer tableviwer_dependencies;

	public Text text_API_description;
	public Label infoLabel;

	private Model_Base Current_Focused_API;
	private GridLayout gl;
	private GridData gd;
	private Text text_title;
	private Text text_description;

	public Label title_description;
	public boolean isDependClear = true;
	public boolean did_save_success = true;

	public Dialog_Service_Edit(Shell parentShell) {
		super(parentShell);
	}

	@Override
	public void create() {
		super.create();
		setTitle("eDrone Service");
		setMessage("서비스 생성/ 편집", IMessageProvider.INFORMATION);
	}

	@Override
	protected void createButtonsForButtonBar(Composite parent) {
		// TODO Auto-generated method stub
//		Button btn_saveas = super.createButton(parent, IDialogConstants.FINISH_ID, IDialogConstants.FINISH_LABEL, true);
		Button btn_cancel = super.createButton(parent, IDialogConstants.CANCEL_ID, IDialogConstants.CANCEL_LABEL, true);
		Button btn_save = super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);
//		btn_saveas.setText("Save as..");
		btn_cancel.setText("Cancel");
		btn_save.setText("Save");
	}

	@Override
	protected Control createDialogArea(Composite parent) {

		getShell().setSize(800, 1000);
		Composite area = (Composite) super.createDialogArea(parent);
		Composite container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(3, false);
		layout.numColumns = 4;
		container.setLayout(layout);
		layout.marginTop = 20;
		layout.marginBottom = 10;
		layout.marginLeft = 20;
		layout.marginRight = 20;
		TabFolder tf = new TabFolder(container, SWT.NULL | SWT.V_SCROLL);
		GridData gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 4;
		gd.heightHint = 500;
		gd.grabExcessHorizontalSpace = true;
		gd.grabExcessVerticalSpace = true;
		tf.setLayoutData(gd);

		TabItem ti3 = new TabItem(tf, SWT.BORDER);
		ti3.setText("API 호출 관리");
		ti3.setControl(new tabService(tf, SWT.SHADOW_ETCHED_IN));

		TabItem ti2 = new TabItem(tf, SWT.BORDER);
		ti2.setText("의존 정보 조회");
		ti2.setControl(new tabDependencies(tf, SWT.SHADOW_ETCHED_IN));

		load_service();
		return area;
	}

	private class tabService extends Composite {

		tabService(Composite c, int style) {
			super(c, style);
			GridLayout gl = new GridLayout(3, false);
			gl.numColumns = 4;
			this.setLayout(gl);

			gd = new GridData(GridData.FILL_HORIZONTAL);
			gd.grabExcessHorizontalSpace = true;

			Label label_title = new Label(this, SWT.NULL);

			label_title.setText("Service Name");
			label_title.setLayoutData(gd);

			text_title = new Text(this, SWT.BORDER);
			gd = new GridData(GridData.FILL_HORIZONTAL);
			gd.heightHint = 50;
			gd.grabExcessHorizontalSpace = true;
			gd.horizontalSpan = 3;
			text_title.setLayoutData(gd);
			text_title.setText(Global.Service_Selected.name);

			Label label_description = new Label(this, SWT.NULL);
			gd = new GridData(GridData.FILL_HORIZONTAL);
			gd.grabExcessHorizontalSpace = true;
			label_description.setText("Service Description");
			label_description.setLayoutData(gd);

			text_description = new Text(this, SWT.BORDER | SWT.MULTI | SWT.WRAP);
			gd = new GridData(GridData.FILL_BOTH);
			gd.heightHint = 30;
			gd.horizontalSpan = 3;
			text_description.setLayoutData(gd);
			text_description.setText(Global.Service_Selected.description);

			gl = new GridLayout();
			GridData gd = new GridData(GridData.FILL_BOTH);
			gd.verticalSpan = 2;
			Group g_availabe = new Group(this, SWT.BORDER);
			g_availabe.setText("가용 API 리스트");
			g_availabe.setLayout(gl);
			g_availabe.setLayoutData(gd);
			new Tree_AvailableAPIs_Edit(g_availabe);

			createArrowButton(this, SWT.RIGHT, sel_listener_button_move_right, 1);

			gl = new GridLayout();
			gl.numColumns = 2;

			gd = new GridData(GridData.FILL_BOTH);
			gd.horizontalSpan = 2;
			gd.verticalSpan = 2;
			Group g_selected = new Group(this, SWT.BORDER);
			g_selected.setText("선택된 API 리스트 및 정보");

			g_selected.setLayout(gl);
			g_selected.setLayoutData(gd);
			new Table_SelectedAPI_Edit(g_selected);
			createArrowButton(g_selected, SWT.UP, sel_listener_move_up, 2);

			createArrowButton(this, SWT.LEFT, sel_listner_button_move_left, 1);

			createArrowButton(g_selected, SWT.DOWN, sel_listener_move_down, 2);

			gd = new GridData(GridData.FILL_HORIZONTAL);
			gd.horizontalSpan = 2;
			gd.horizontalAlignment = SWT.BEGINNING;
			title_description = new Label(g_selected, SWT.NULL);
			title_description.setText("API 설명 입력");
			title_description.setLayoutData(gd);

			text_API_description = new Text(g_selected, SWT.WRAP | SWT.MULTI | SWT.BORDER | SWT.V_SCROLL);

			gd = new GridData(GridData.FILL_BOTH);
//			gd.heightHint = 30;
			gd.horizontalSpan = 2;
			gd.horizontalAlignment = SWT.FILL;
			gd.grabExcessHorizontalSpace = true;

			text_API_description.setLayoutData(gd);

			text_API_description.addFocusListener(new FocusListener() {

				@Override
				public void focusLost(FocusEvent e) {

//					IStructuredSelection iss = tableviewer_selected_api.getStructuredSelection();

					if (Current_Focused_API != null) {

						Current_Focused_API.description = text_API_description.getText();

					}
				}

				@Override
				public void focusGained(FocusEvent e) {
					IStructuredSelection iss = tableviewer_selected_api.getStructuredSelection();
					if (iss != null) {

						Model_Base mb = (Model_Base) iss.getFirstElement();
						Current_Focused_API = mb;

					}

				}
			});
			// Label4.setText("Input Parameter");

			Group GParam = new Group(this, style);
			gd = new GridData(GridData.FILL_BOTH);
			gd.horizontalSpan = 4;
			gd.horizontalAlignment = SWT.FILL;
			gd.grabExcessHorizontalSpace = true;
			GParam.setLayoutData(gd);

			gl = new GridLayout();
			gl.numColumns = 7;
			GParam.setLayout(gl);
			GParam.setText("API 파라미터");

			Label Label1 = new Label(GParam, SWT.CENTER);
			gd = new GridData(GridData.FILL_HORIZONTAL | GridData.CENTER);
			gd.horizontalSpan = 7;
			gd.grabExcessHorizontalSpace = true;

			Label1.setLayoutData(gd);
			Label1.setText("Input Parameter");

			new Table_ParamIn(GParam);

			Label Label2 = new Label(GParam, SWT.CENTER);
			gd = new GridData(GridData.FILL_HORIZONTAL | GridData.CENTER);
			gd.horizontalSpan = 7;
			gd.grabExcessHorizontalSpace = true;

			Label2.setLayoutData(gd);
			Label2.setText("Output Parameter");
			new Table_ParamOut(GParam);

			this.addListener(SWT.Traverse, new Listener() {

				@Override
				public void handleEvent(Event event) {

					if (event.detail == SWT.TRAVERSE_RETURN) {

						return;
					}
				}
			});

		}
	}

	public void load_service() {

		Global.dialog_opened.tableviewer_selected_api.setInput(Global.Service_Selected);

		List<Model_Base> list = Global.Service_Selected.api_list;
		if (list.size() > 0) {
			Global.dialog_opened.tableviewer_selected_api.setSelection(new StructuredSelection(list.get(0)));
		}

		Global.isDependClear = true;
		ArrayList<String> duplicate_check = new ArrayList<String>();
		ArrayList<Model_Base> list_depend = new ArrayList<Model_Base>();
		for (Model_Base model : list) {

			String parentName = model.parent.name;
			if (duplicate_check.contains(parentName) == false) {
				duplicate_check.add(parentName);
				list_depend.add(model.parent);
			}

			model.parent.get_dpendencies();

		}
		Model_Service model_service = (Model_Service) Global.dialog_opened.tableviwer_dependencies.getInput();

		model_service.depend_list = list_depend;

		Global.dialog_opened.tableviwer_dependencies.setInput(model_service);

		Global.validate_service(false);

	}

	private class tabDependencies extends Composite {

		tabDependencies(Composite c, int style) {

			super(c, style);
			GridLayout gl = new GridLayout();
			gl.numColumns = 3;
			this.setLayout(gl);

			GridData gd = new GridData(GridData.FILL_HORIZONTAL);
			gd.horizontalSpan = 3;
			gd.grabExcessHorizontalSpace = true;
			new Table_Dependencies(this);

		}
	}

	public void createArrowButton(Composite parent, int alignment, SelectionListener selectionListener, int choice) {

		GridData gd = null;
		switch (choice) {
		case 1:
			gd = new GridData(GridData.VERTICAL_ALIGN_CENTER | GridData.FILL_VERTICAL | GridData.GRAB_VERTICAL);
			break;
		case 2:
			gd = new GridData();
			break;
		}

		gd.heightHint = 70;
		final Button arwBtn = new Button(parent, SWT.ARROW);
		arwBtn.setAlignment(alignment);
		arwBtn.addSelectionListener(selectionListener);
		arwBtn.setLayoutData(gd);

	}

	SelectionListener sel_listener_move_up = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {

			TableViewer viewer = Global.dialog_opened.tableviewer_selected_api;
			int index = viewer.getTable().getSelectionIndex();
			if (index == -1)
				return;

			List<Model_Base> table_list = ((Model_Service) viewer.getInput()).api_list;
			if (index != 0) {

//				 Global.logger(table_list.get(index).priority.toString());
//				 Global.logger(table_list.get(index-1).priority.toString());

				if (table_list.get(index).priority < table_list.get(index - 1).priority) {

					MessageDialog.openError(getShell(), "Warning", "API" + table_list.get(index).name + "은 "
							+ table_list.get(index - 1) + "보다 먼저 호출될 수 없습니다. " + "\n 우선순위 (priority)를 확인하세요.");
					return;
				}

				table_list.get(index).order -= 1;
				table_list.get(index - 1).order += 1;
				Collections.swap(table_list, index - 1, index);
				viewer.refresh();
				viewer.setSelection(new StructuredSelection(table_list.get(index - 1)));
				viewer.getTable().showSelection();
			}
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}

	};

	SelectionListener sel_listener_move_down = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {

			TableViewer viewer = Global.dialog_opened.tableviewer_selected_api;
			int index = viewer.getTable().getSelectionIndex();
			if (index == -1)
				return;

			List<Model_Base> table_list = ((Model_Service) viewer.getInput()).api_list;

			if (index < table_list.size() - 1) {

				if (table_list.get(index).priority > table_list.get(index + 1).priority) {

					MessageDialog.openError(getShell(), "Warning", "API" + table_list.get(index).name + "은 "
							+ table_list.get(index + 1) + "보다 늦게 호출될 수 없습니다." + "\n 우선순위 (priority)를 확인하세요.");
					return;
				}

				table_list.get(index).order += 1;
				table_list.get(index + 1).order -= 1;
				Collections.swap(table_list, index + 1, index);
				viewer.refresh();
				viewer.setSelection(new StructuredSelection(table_list.get(index + 1)));
				viewer.getTable().showSelection();
			}
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}

	};

	SelectionListener sel_listner_button_move_left = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {

			TableViewer viewer = Global.dialog_opened.tableviewer_selected_api;
			int index = viewer.getTable().getSelectionIndex();
			if (index == -1)
				return;

			List<Model_Base> table_list = ((Model_Service) viewer.getInput()).api_list;

			table_list.remove(index);
			if (table_list.isEmpty()) {
				viewer.refresh();

				Global.dialog_opened.isDependClear = true;

				Model_Service model_service = (Model_Service) Global.dialog_opened.tableviwer_dependencies.getInput();

				model_service.depend_list = new ArrayList<Model_Base>();

				Global.dialog_opened.tableviwer_dependencies.setInput(model_service);
				Global.validate_service(false);
				return;
			}
			for (int i = 0; i < table_list.size(); i++) {
				table_list.get(i).order = i + 1;
			}

			Global.dialog_opened.tableviewer_selected_api
					.setSelection(new StructuredSelection(table_list.get(index - 1)));
			viewer.refresh();

			Global.dialog_opened.isDependClear = true;

			ArrayList<String> duplicate_check = new ArrayList<String>();
			ArrayList<Model_Base> list_depend = new ArrayList<Model_Base>();
			for (Model_Base model : table_list) {

				String parentName = model.parent.name;
				if (duplicate_check.contains(parentName) == false) {
					duplicate_check.add(parentName);
					list_depend.add(model.parent);
					model.parent.get_dpendencies();
				}

			}

			Model_Service model_service = (Model_Service) Global.dialog_opened.tableviwer_dependencies.getInput();

			model_service.depend_list = list_depend;
			Global.dialog_opened.tableviwer_dependencies.refresh();

			Global.validate_service(false);

		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}

	};
	SelectionListener sel_listener_button_move_right = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {

			TreeViewer viewer = Global.dialog_opened.treeviewer_availabe_api;

			IStructuredSelection thisSelection = (IStructuredSelection) viewer.getSelection();
			if (thisSelection == null)
				return;
			Object selectedNode = thisSelection.getFirstElement();
			Model_Base model_old = (Model_Base) selectedNode;
			Model_Base model = new Model_Base(model_old);

			if (model.isAPI) {

				Model_Service ms = (Model_Service) Global.dialog_opened.tableviewer_selected_api.getInput();

				List<Model_Base> list = ms.api_list;

				ArrayList<Model_Base> new_list = new ArrayList<Model_Base>();

				boolean inserted = false;
				int i = 0;
				if (list.size() > 0) {

					for (Model_Base mb : list) {
						if (model.priority.compareTo(mb.priority) > 0 && inserted == false) {
							new_list.add(model);
							model.order = i + 1;
							inserted = true;
							i++;
						}
						new_list.add(mb);
						mb.order = i + 1;
						i++;
					}
				}
				if (inserted == false) {
					model.order = i + 1;
					new_list.add(model);
				}
				ms.api_list = new_list;

				Global.dialog_opened.tableviewer_selected_api.setInput(ms);

				Global.dialog_opened.tableviewer_selected_api.setSelection(new StructuredSelection(model), true);
				Global.validate_service(false);

				Integer last_index = Global.dialog_opened.tableviewer_selected_api.getTable().getItemCount() - 1;

				Global.dialog_opened.tableviewer_selected_api.setSelection(
						new StructuredSelection(Global.dialog_opened.tableviewer_selected_api.getElementAt(last_index)),
						true);

				Global.dialog_opened.isDependClear = true;

				Model_Service model_service = (Model_Service) Global.dialog_opened.tableviwer_dependencies.getInput();

				for (Model_Base m : model_service.depend_list) {
					if (model.parent.name.equals(m.name)) {
						return;
					}
				}
				model_service.depend_list.add(model.parent);
				model.parent.get_dpendencies();

				Global.dialog_opened.tableviwer_dependencies.setInput(model_service);
			}

		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}

	};

	@Override
	protected boolean isResizable() {
		return true;
	}

	@Override
	protected void okPressed() {
		did_save_success = true;
		if (treeviewer_availabe_api.getControl().isFocusControl()
				|| tableviwer_dependencies.getControl().isFocusControl()
				|| tableviewer_selected_api.getControl().isFocusControl()
				|| tableviewer_paramin.getControl().isFocusControl()
				|| tableviewer_paramout.getControl().isFocusControl())

			return;
		Global.Service_Selected.name = text_title.getText();

		if (Global.Service_Selected.name.replace(" ", "").length() < 1) {

			MessageDialog.openWarning(getShell(), "Warning", "서비스 이름을 입력하세요");
			return;
		}

		Global.Service_Selected.description = text_description.getText();

		String save_path = Global.Service_Selected.file_path;
//		File ServiceRootfile = new File(Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/services");

		if (save_path.length() < 1) {
			DirectoryDialog dir_dialog = new DirectoryDialog(getShell());

			dir_dialog.setFilterPath(Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/services");

			save_path = dir_dialog.open();
			if (save_path == null)
				return;

			save(save_path, true);
			MessageDialog.openInformation(getShell(), "eDrone", "서비스 생성완료");
			Global.treeviewerService.refresh();

			super.okPressed();
			return;
		} else {

			File f = new File(save_path);
			File fileParent = f.getParentFile();
			File toFile = new File(fileParent.getAbsolutePath() + "/" + Global.Service_Selected.name + ".xml");
			if (f.getName().equals(toFile.getName()) == false) {
				if (toFile.exists()) {
					MessageDialog.openError(getShell(), "eDrone", "해당 이름의 서비스가 이미 존재합니다.");
					return;
				}
			}
			f.renameTo(toFile);

			save(toFile.getAbsolutePath(), false);

			Global.treeviewerService.refresh();
		}

		if (did_save_success == false) {
			MessageDialog.openError(getShell(), "eDrone", "서비스 수정 중 오류 ");
		}
		super.okPressed();
	}

	private void save(String path, boolean isDirectory) {

		if (path != null) {
			File target = null;
			if (isDirectory) {
				target = new File(path + "/" + Global.Service_Selected.name + ".xml");
			} else {
				target = new File(path);
			}
			DocumentBuilderFactory factory;
			DocumentBuilder builder;
			Document doc;

			try {
				factory = DocumentBuilderFactory.newInstance();
				factory.setNamespaceAware(true);
				builder = factory.newDocumentBuilder();
				doc = builder.newDocument();

				Element rootElement = doc.createElement("section");
				rootElement.setAttribute("name", Global.Service_Selected.name);
				rootElement.setAttribute("description", Global.Service_Selected.description);

				doc.appendChild(rootElement);
				Element elt = doc.createElement("topic");
				elt.setAttribute("name", "Target");
				Element elf = doc.createElement("field");
				elf.setAttribute("type", "uint16");
				elf.setAttribute("name", "target_seq_no");

				Element elf2 = doc.createElement("field");
				elf2.setAttribute("type", "bool");
				elf2.setAttribute("name", "is_global");
				Element elf3 = doc.createElement("field");
				elf3.setAttribute("type", "float64");
				elf3.setAttribute("name", "x_lat");
				Element elf4 = doc.createElement("field");
				elf4.setAttribute("type", "float64");
				elf4.setAttribute("name", "y_long");
				Element elf5 = doc.createElement("field");
				elf5.setAttribute("type", "float64");
				elf5.setAttribute("name", "z_alt");
				Element elf6 = doc.createElement("field");
				elf6.setAttribute("type", "bool");
				elf6.setAttribute("name", "reached");

				elt.appendChild(elf);
				elt.appendChild(elf2);
				elt.appendChild(elf3);
				elt.appendChild(elf4);
				elt.appendChild(elf5);
				elt.appendChild(elf6);

				Element elt2 = doc.createElement("topic");
				elt2.setAttribute("name", "Phase");

				Element elf7 = doc.createElement("field");
				elf7.setAttribute("type", "String");
				elf7.setAttribute("name", "phase");

				elt2.appendChild(elf7);

				rootElement.appendChild(elt);
				rootElement.appendChild(elt2);

				for (Model_Base s : Global.Service_Selected.api_list) {

					Element els = doc.createElement("service");
					els.setAttribute("name", s.name);
					els.setAttribute("priority", Double.toString(s.priority));
					els.setAttribute("description", s.description);

					for (Model_Base p : s.child) {

						Element elp = doc.createElement("param");
						elp.setAttribute("name", p.name);
						elp.setAttribute("description", p.description);
						elp.setAttribute("io", p.isIn ? "in" : "out");
						elp.setAttribute("type", p.type);
						elp.setAttribute("default", p.value);
						if (p.isIn) {
							elp.setAttribute("ptype", p.ptype.name());

							if (p.ptype.equals(ParamType.Combo)) {
								elp.setAttribute("back_value", p.back_value);
							}
						}
						els.appendChild(elp);
					}
					rootElement.appendChild(els);
				}

				TransformerFactory transformerFactory = TransformerFactory.newInstance();
				Transformer transformer = transformerFactory.newTransformer();

				transformer.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
				transformer.setOutputProperty(OutputKeys.INDENT, "yes");
				transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");
				DOMSource source = new DOMSource(doc);
				StreamResult result = new StreamResult(new FileOutputStream(target));

//			 파일로 쓰지 않고 콘솔에 찍어보고 싶을 경우 다음을 사용 (디버깅용)
//				StreamResult result = new StreamResult(System.out);

				transformer.transform(source, result);

			} catch (Exception e) {
				did_save_success = false;
				e.printStackTrace();
				return;
			}
		}
	}

}
