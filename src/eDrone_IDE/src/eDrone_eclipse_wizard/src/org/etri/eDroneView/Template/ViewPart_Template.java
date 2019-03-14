package org.etri.eDroneView.Template;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IFolder;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;
import org.eclipse.ui.IWorkbenchPage;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.ide.IDE;
import org.eclipse.ui.part.ViewPart;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Section;
import org.etri.eDroneModel.Model_Service;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class ViewPart_Template extends ViewPart {

	Text text_template_content;
	public Tree_AvailableAPIs_Template tree;
	public Table_Section table;
	public Text annotation;
	public Text text_content;
	public Model_Service model_Service = null;
	public Composite container;

	@Override
	public void createPartControl(Composite parent) {

		GridLayout gl = new GridLayout();

		container = new Composite(parent, SWT.V_SCROLL);
		gl.numColumns = 4; // numColumns 는 위저드 화면을 세로로 몇등분할지를 정한다.
		gl.marginLeft = 30;
		gl.marginRight = 30;
		gl.verticalSpacing = 10;
		gl.marginTop = 20;
		gl.marginBottom = 20;
		gl.horizontalSpacing = 10;
		gl.makeColumnsEqualWidth = true;
		container.setLayout(gl);
		Global.initialize();

		Group g_package = new Group(container, SWT.BORDER);

		gl = new GridLayout();
		gl.numColumns = 2;
		gl.marginBottom = 10;
		gl.marginTop = 5;
		gl.marginLeft = 5;
		gl.marginRight = 5;
		gl.horizontalSpacing = 5;
		gl.verticalSpacing = 5;

		gl.makeColumnsEqualWidth = true;
		gl.numColumns = 2;
		g_package.setLayout(gl);
		g_package.setText("Package Templates");
		GridData gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 4;
		g_package.setLayoutData(gd);

		Button btn_package = new Button(g_package, SWT.PUSH);
		btn_package.setText("package.xml");
		gd = new GridData(GridData.FILL_HORIZONTAL);
		btn_package.setLayoutData(gd);
		btn_package.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
					OpenTemplateEditor(Global.CATKIN_WORKSPACE_PATH + "/" + Global.DEPNEDING_FILES_PATH
							+ "/template/package/package.tpl", "package.tpl");
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		Button btn_cmake = new Button(g_package, SWT.PUSH);
		btn_cmake.setText("CMakeList.txt");
		gd = new GridData(GridData.FILL_HORIZONTAL);
		btn_cmake.setLayoutData(gd);
		btn_cmake.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				OpenTemplateEditor(Global.CATKIN_WORKSPACE_PATH + "/" + Global.DEPNEDING_FILES_PATH
						+ "/template/package/cmakelist.tpl", "CmakeList.tpl");
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});
		Button btn_launch = new Button(g_package, SWT.PUSH);
		btn_launch.setText(".launch");
		gd = new GridData(GridData.FILL_HORIZONTAL);
		btn_launch.setLayoutData(gd);
		btn_launch.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				OpenTemplateEditor(Global.CATKIN_WORKSPACE_PATH + "/" + Global.DEPNEDING_FILES_PATH
						+ "/template/package/launch.tpl", "launch.tpl");
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});
		Button btn_param = new Button(g_package, SWT.PUSH);
		btn_param.setText("params.h");
		gd = new GridData(GridData.FILL_HORIZONTAL);
		btn_param.setLayoutData(gd);
		btn_param.addSelectionListener(new SelectionListener() {
			@Override
			public void widgetSelected(SelectionEvent e) {
				OpenTemplateEditor(Global.CATKIN_WORKSPACE_PATH + "/" + Global.DEPNEDING_FILES_PATH
						+ "/template/package//params.xml", "params.xml");
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});
		Group g_source = new Group(container, SWT.BORDER);

		gl = new GridLayout();
		gl.numColumns = 2;
		gl.marginBottom = 10;
		gl.marginTop = 5;
		gl.marginLeft = 5;
		gl.marginRight = 5;
		gl.horizontalSpacing = 5;
		gl.verticalSpacing = 5;

		gl.makeColumnsEqualWidth = true;
		gl.numColumns = 2;
		g_source.setLayout(gl);
		g_source.setText("API Templates");
		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 4;
		g_source.setLayoutData(gd);

		Tree_AvailableAPIs_Template tat = new Tree_AvailableAPIs_Template(g_source);

		Model_Base global_model = new Model_Base("Global", Global.MODEL_API_ROOT);
		global_model.isAPI = true;
		Global.MODEL_API_ROOT.child.add(0, global_model);

		tat.viewer.setInput(Global.MODEL_API_ROOT);
		tat.viewer.expandAll();

		gd = new GridData();
		Label dummyLabel = new Label(g_source, SWT.NULL);
		dummyLabel.setLayoutData(gd);

		Button button_Section_Setting = new Button(g_source, SWT.PUSH);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.heightHint = 40;
		button_Section_Setting.setLayoutData(gd);
		button_Section_Setting.setText("Sections...");
		button_Section_Setting.addSelectionListener(new SelectionListener() {
			@Override
			public void widgetSelected(SelectionEvent e) {

				read_sections(new File(Global.CATKIN_WORKSPACE_PATH + "/" + Global.DEPNEDING_FILES_PATH
						+ "/template/source/section.xml"));
				Dialog_Section ds = new Dialog_Section(getSite().getShell());
				ds.open();

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});
	}

	public void OpenTemplateEditor(String tplPath, String name) {

		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();

		IProject project = root.getProject("[eDrone Template Edit Support]");
		try {
			if (project.exists() == false) {
				project.create(null);
				project.open(null);
			}
		} catch (CoreException e1) {
			e1.printStackTrace();
		}

		IFolder folder = project.getFolder("package");
		try {
			if (folder.exists() == false) {
				folder.create(false, true, null);
			}
		} catch (CoreException e) {
			e.printStackTrace();
		}

		File f = new File(tplPath);

		if (f.exists() == false) {

			try {
				f.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}

		}
		IFile file = folder.getFile(name);
		try {
			if (file.exists() == false) {
				file.createLink(new URI(tplPath), IResource.BACKGROUND_REFRESH, null);
			}

			IWorkbenchPage page = PlatformUI.getWorkbench().getActiveWorkbenchWindow().getActivePage();

			if (page != null) {
				try {
					IDE.openEditor(page, file, true);

				} catch (PartInitException e) {
					e.printStackTrace();
				}
			}
		} catch (CoreException | URISyntaxException e) {
			e.printStackTrace();
		}
	}

	private void read_sections(File f) {

		DocumentBuilderFactory factory;
		DocumentBuilder builder = null;
		Document doc = null;

		Global.list_sections = new ArrayList<Model_Section>();

		try {
			factory = DocumentBuilderFactory.newInstance();
			factory.setNamespaceAware(true);
			builder = factory.newDocumentBuilder();
			doc = builder.parse(f);

			Element element_root = doc.getDocumentElement();
			NodeList nodelist_section = element_root.getElementsByTagName("section");

			for (int i = 0; i < nodelist_section.getLength(); i++) {
				Node node = nodelist_section.item(i);
				if (node.getNodeType() == Node.ELEMENT_NODE) {

					Element element = (Element) nodelist_section.item(i);
					String secname = element.getAttribute("name");
					String annotation = element.getAttribute("annotation");
					Model_Section model_sec = new Model_Section(secname, annotation);
					model_sec.order = i + 1;
					if (secname.equals("main") || secname.equals("end")) {
						(model_sec.content_stringbuilder).append(element.getTextContent());
					}
					Global.list_sections.add(model_sec);
				}
			}

		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	@Override
	public void setFocus() {

		Global.view_template = this;
		Global.initialize();
	}

}