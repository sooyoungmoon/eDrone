package org.etri.eDrone;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.inject.Inject;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IResourceChangeEvent;
import org.eclipse.core.resources.IResourceChangeListener;
import org.eclipse.core.resources.IResourceDelta;
import org.eclipse.core.resources.IWorkspace;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.ICoreRunnable;
import org.eclipse.core.runtime.jobs.Job;
import org.eclipse.jface.resource.ImageDescriptor;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.swt.graphics.Image;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Section;
import org.etri.eDroneModel.Model_Service;
import org.etri.eDroneModel.Model_Type;
import org.etri.eDroneView.Service.Dialog_Service_Edit;
import org.etri.eDroneView.Service.ViewPart_Service;
import org.etri.eDroneView.Template.ViewPart_Template;
import org.etri.eDroneWizard.WizardPage1;
import org.etri.eDroneWizard.WizardPage2;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

/**
 * @Class Global
 * @brief static 변수들 집합
 * @author etri
 * @version 1.0.0
 * 
 */

public class Global {

	// 위자드 첫번째 페이지 (서비스 선택)
	public static WizardPage1 wizard_page1 = null;
	// 위자드 두번째 페이지 (API 선택)
	public static WizardPage2 wizard_page2 = null;

	// 서포타 뷰파트
	public static ViewPart_Service view_service = null;
	public static ViewPart_Template view_template = null;

	// Catkin 워크스페이스 경로.
	// WizardPage1 initialize() 함수에서 초기화
	public static String CATKIN_WORKSPACE_PATH = "";
	public static String CROSS_CATKIN_WORKSPACE_PATH = "";
	// ROS Root 경로.
	// WizardPage1 initialize() 함수에서 초기화
	public static String ROS_ROOT_PATH = "";
	// 템플릿 파일, 서비스 파일등이 위치한 디렉토리 이름
	public static String DEPNEDING_FILES_PATH = "edrone_mc_support";
	public static String INCLUDE_HEADER_PATH = "/opt/ros/kinetic/include/";

	public static Model_Base MODEL_API_ROOT;
	public static boolean isDependClear = true;

	public static Job job;
	// 이미지 static 변수들
	public static final ImageDescriptor imagedescriptor_Logo_2x = Activator.getImageDescriptor("icons/logo_@2x.png");
	public static final Image image_Logo = Activator.getImageDescriptor("icons/logo_@1x.png").createImage();
	public static final Image image_Project = Activator.getImageDescriptor("icons/img_project.png").createImage();
	public static final Image image_API = Activator.getImageDescriptor("icons/img_api.png").createImage();
	public static final Image image_Folder = Activator.getImageDescriptor("icons/img_folder.png").createImage();
	public static final Image image_Service = Activator.getImageDescriptor("icons/img_service.png").createImage();
	public static final Image image_Checked = Activator.getImageDescriptor("icons/img_checked.gif").createImage();
	public static final Image image_UnChecked = Activator.getImageDescriptor("icons/img_unchecked.gif").createImage();
	public static final Image image_Accepted = Activator.getImageDescriptor("icons/img_accept.png").createImage();
	public static final Image image_Denied = Activator.getImageDescriptor("icons/img_error.png").createImage();
	public static final Image image_info = Activator.getImageDescriptor("icons/img_info.png").createImage();
	public static final Image image_Warn = Activator.getImageDescriptor("icons/img_warning.png").createImage();
	public static final Image image_Tree = Activator.getImageDescriptor("icons/img_tree.png").createImage();
	public static final Image image_List = Activator.getImageDescriptor("icons/img_list_numbers.png").createImage();
	public static final Image image_Template = Activator.getImageDescriptor("icons/img_template.png").createImage();
	public static final Image image_Section = Activator.getImageDescriptor("icons/img_section.png").createImage();
	public static final Image image_Add = Activator.getImageDescriptor("icons/img_add.png").createImage();
	public static final Image image_delete = Activator.getImageDescriptor("icons/img_delete.png").createImage();
	public static final Image image_gear = Activator.getImageDescriptor("icons/img_gear.png").createImage();
	public static final Image image_Anchor = Activator.getImageDescriptor("icons/img_anchor.png").createImage();
	public static final Image image_Import = Activator.getImageDescriptor("icons/application_get.png").createImage();
	public static final Image image_Export = Activator.getImageDescriptor("icons/application_put.png").createImage();
	public static final Image image_Global = Activator.getImageDescriptor("icons/img_global.png").createImage();
	// 가용 API들의 의존성 등을 보다 효과적으로 접근하기 위해 만든 전역 해시맵
	// key : API 이름
	// value : API 정보를 담은 MODEL 객체
	public static HashMap<String, Model_Base> hashmap_api = new HashMap<String, Model_Base>();
	// 최종 생성될 서비스의 정보를 담은 객체
	public static Model_Service Service_Selected = null;
	// 첫번째 화면에서 Next 버튼을 누르기 전에 사용자가 서비스를 클릭할 경우 정보를 잠시 담아두었다가
	// Next 버튼이 눌리면 Model_Service 변수로 옮기기 위해 잠깐 붙들고 있는 객체
	public static Model_Service Service_Temp = null;
	// 가용 프로젝트 이름의 목록. 두펀째 페이지 의존정보 탭에서 의존 정보 체크할 때 사용된다.
	public static ArrayList<String> list_pacakges = new ArrayList<String>();
	// 탬플릿 섹션 리스트
	public static ArrayList<Model_Section> list_sections = new ArrayList<Model_Section>();
	public static ArrayList<Model_Section> list_temp_sections = new ArrayList<Model_Section>();
	public static HashMap<String, ArrayList<String>> hashmap_headers;
	public static HashMap<String, Model_Type> hashmap_type = new HashMap<String, Model_Type>();

	// UI 상에서 지나치게 왼쪽으로 붙은 텍스트에 균일한 공백을 주기 위해 전역으로 선언
	public static String PADDING = "  ";
	public static Dialog_Service_Edit dialog_opened;
	public static File copied_file;

	public static String CURRENT_TARGET_PORT = "";
	public static String CURRENT_TARGET_ADDRESS = "";
	public static String CURRENT_TARGET_PROJECTNAME = "";
	public static String CURRENT_TARGET_USERNAME = "";
	public static String CURRENT_TARGET_PASSWORD = "";

	public static TreeViewer treeviewerService = null;

//	public static Object last_focused;
//	public static Model_Section last_focused_template;

	/////// Global Fuctions //////

	// 출력 제어
	public static void logger(String log) {
		if (false)
			System.out.println("[eDone] : " + log);
	}

	public static void initialize() {
		String workspaceLoc = System.getenv("CATKIN_WS_PATH");
		String cross_workspaceLoc = System.getenv("CROSS_CATKIN_WS_PATH");
		String temp = System.getenv("ROS_ROOT");
		String rosLoc = temp.substring(0, temp.length() - 4);
		String tmp = System.getenv("ROS_PACKAGE_PATH");

		System.out.println(tmp);
		Global.CATKIN_WORKSPACE_PATH = workspaceLoc + "/";
		Global.CROSS_CATKIN_WORKSPACE_PATH = cross_workspaceLoc + "/";
		Global.ROS_ROOT_PATH = rosLoc;

		Global.logger("CATKIN_WS_PATH : " + Global.CATKIN_WORKSPACE_PATH);
		Global.logger("ROS_PATH : " + Global.ROS_ROOT_PATH);

		Global.hashmap_api = new HashMap<String, Model_Base>();
		Global.list_pacakges = new ArrayList<String>();

		MODEL_API_ROOT = new Model_Base("root", null);
		Model_Base Model_Srvice_Catkin_ws = Global.makeModel("Catkin_WS", Global.CATKIN_WORKSPACE_PATH + "/src");
		Model_Base Model_Seervice_ROS = Global.makeModel("ROS", Global.ROS_ROOT_PATH);

		MODEL_API_ROOT.child.add(Model_Srvice_Catkin_ws);
		MODEL_API_ROOT.child.add(Model_Seervice_ROS);

		Global.Service_Selected = null;

		// 헤더정보 관련 해쉬 맵 업데이트
		Global.hashmap_headers = new HashMap<String, ArrayList<String>>();
		FilenameFilter filter_header = new FilenameFilter() {
			public boolean accept(File dir, String name) {
				return name.toLowerCase().endsWith(".h");
			}
		};
		File include_root = new File(Global.INCLUDE_HEADER_PATH);
		Global.find_headers(include_root, filter_header);
		Global.read_types();
		read_sections(new File(CATKIN_WORKSPACE_PATH + "/" + DEPNEDING_FILES_PATH + "/template/source/section.xml"));

		IWorkspace workspace = ResourcesPlugin.getWorkspace();
		IResourceChangeListener rcl = new IResourceChangeListener() {
			public void resourceChanged(IResourceChangeEvent event) {

				if (event != null) {
					IResource rsrc = event.getResource();
					if (rsrc instanceof IProject) {
						int res = event.getType();
						if (res == IResourceChangeEvent.PRE_DELETE) {
							IProject ip = (IProject) rsrc;
							String name = ip.getName();
							name = name.split("@")[0];

							File fileSrc = new File(CATKIN_WORKSPACE_PATH + "/src/" + name);
//							File fileBuild = new File(CATKIN_WORKSPACE_PATH + "/build/" + name);
							File fileCrossSrc = new File(CROSS_CATKIN_WORKSPACE_PATH + "/src/" + name);

							if (fileSrc.exists()) {
								fileSrc.delete();
							}
							if (fileCrossSrc.exists()) {
								fileCrossSrc.delete();
							}
						}
					}
				}
			}
		};
		workspace.addResourceChangeListener(rcl);

	}

	
	// params.h의 다양한 타입 지원을 위해 #include 문을 추가해야하는데,
	// /opt/ros/kinetic/include 디렉토리 하부의 헤더 파일을 전수조사해야한다.

	public static void find_headers(File file, FilenameFilter filter) {

		for (File f : file.listFiles()) {
			if (f.isDirectory()) {
				find_headers(f, filter);
			}
		}

		for (File f : file.listFiles(filter)) {

			String name = f.getName();

			if (hashmap_headers.containsKey(name) == false) {
				hashmap_headers.put(name, new ArrayList<String>());
			}
			hashmap_headers.get(name).add(f.getAbsolutePath());

		}

	}

	// 가용 API를 확인하는 작업은 위자드 실행시마다 해야하고, 서포터에서도 호출할 수 있어야 하므로 static 함수로 만듦
	public static Model_Base makeModel(String name, String path) {

		Model_Base model_root = new Model_Base(name, null);
		File file_root = new File(path);

		FilenameFilter filter_srv = new FilenameFilter() {
			public boolean accept(File dir, String name) {
				return name.toLowerCase().endsWith(".srv");
			}
		};

		for (File file_project : file_root.listFiles()) {

			if (file_project.isDirectory() == false)
				continue;

			String projectName = file_project.getName();
			logger("===========================================");
			logger("");
			logger("$ " + projectName);
			logger("");

			Model_Base model_project = new Model_Base(projectName, model_root);
			model_project.location = file_project.getAbsolutePath();
			getAPIList(file_project, model_project, filter_srv);

			if (model_project.child.size() > 0) {
				list_pacakges.add(model_project.name);
				model_root.child.add(model_project);
			}
		}
		return model_root;
	}

	// MakeModel에 의해 재귀적으로 호출된다.
	static void getAPIList(File file, Model_Base parent, FilenameFilter filter) {

		String line = "";
		for (File sub_f : file.listFiles()) {

			if (sub_f.getName().equals("package.xml")) {
				parent.isProject = true;
				list_pacakges.add(parent.name);
			}

			if (sub_f.isDirectory()) {
				String dirName = sub_f.getName();
				if (dirName.equals("srv")) {
					getAPIList(sub_f, parent, filter);
					continue;
				}
				Model_Base model_project = new Model_Base(dirName, parent);
				model_project.location = sub_f.getAbsolutePath();
				getAPIList(sub_f, model_project, filter);

				if (model_project.child.size() > 0) {
					parent.child.add(model_project);
				}
			}
		}

		for (File sub_f : file.listFiles(filter)) {

			String apiName = sub_f.getName();
			apiName = apiName.substring(0, apiName.length() - 4);
			Model_Base model_api = new Model_Base(apiName, parent);
			model_api.isAPI = true;

			logger("\t" + apiName);

			try {
				FileReader reader = new FileReader(sub_f);
				@SuppressWarnings("resource")
				BufferedReader buf_reader = new BufferedReader(reader);

				boolean isParamIn = true;

				while ((line = buf_reader.readLine()) != null) {
					if (line.length() < 1)
						continue;
					if (line.indexOf("---") != -1)
						isParamIn = false;

					String[] tokens = line.split("\\s+");

					if (tokens.length < 2)
						continue;
					if (tokens[0].indexOf("#") != -1 || tokens[1].indexOf("#") != -1)
						continue;

					String type = tokens[0];
					String name = tokens[1];

					if (name.indexOf("=") != -1) {
						name = name.split("=")[0];
					}

					model_api.child.add(new Model_Base(isParamIn, type, name, model_api));
					model_api.name_cache.add(type + name);
				}

				if (model_api.child.size() > 0) {
					hashmap_api.put(apiName, model_api);
					parent.child.add(model_api);
				}

			} catch (IOException e) {
				e.printStackTrace();
			}

		}
	}

	public static void validate_service(boolean isWizard) {

		boolean is_all_api_checked = true;
		List<Model_Base> list = Global.Service_Selected.api_list;
		if (list.size() > 0) {

			for (Model_Base api : list) {

				for (Model_Base param : api.child) {

					if (param.isIn == false)
						continue;

					if (param.hasCondition) {
						param.check_conditions();
						if (param.isConditionSatisfied == false) {
							is_all_api_checked = false;
						}
					} else {
						param.check_conditions();
					}
					if (param.isConditionSatisfied == false) {
						is_all_api_checked = false;
					}

				}
			}
		}

		if (isWizard) {
			if (is_all_api_checked == false) {
				Global.wizard_page2.updateStatus("입력되지 않은 파라미터 값이 있습니다");
			} else {
				Global.wizard_page2.updateStatus(null);
			}

			Global.wizard_page2.setPageComplete(is_all_api_checked);
		} else {

			if (is_all_api_checked == false) {
				Global.dialog_opened.setMessage("파라미터 조건을 확인하세요");
			} else {
				Global.dialog_opened.setMessage("");
			}
		}

	}

	public static void read_types() {

		String path = Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/template/package/params.xml";
		File file = new File(path);

		DocumentBuilderFactory factory;
		DocumentBuilder builder;
		try {
			factory = DocumentBuilderFactory.newInstance();
			builder = factory.newDocumentBuilder();
			Document doc = builder.parse(file);
			Element element_root = doc.getDocumentElement();

			NodeList list_basicType = element_root.getElementsByTagName("basicType");

			for (int i = 0; i < list_basicType.getLength(); i++) {
				Node node = list_basicType.item(i);
				if (node.getNodeType() == Node.ELEMENT_NODE) {

					Element element = (Element) node;
					hashmap_type.put((String) element.getAttribute("from"), new Model_Type(element));
				}
			}
			System.out.println("done");

		} catch (IOException | SAXException | ParserConfigurationException e) {
			e.printStackTrace();
		}

	}

	private static void read_sections(File f) {

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

}
