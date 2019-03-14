package org.etri.eDroneModel;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.etri.eDrone.Global;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

/*
 * 
 * Model_API 객체는 프로젝트/서비스/파라미터 구분 없이 담을 수 있도록 고안됐다.
 * 
 * 모두 이름을 가지며 root를 제외하고는 다른 Model_API parent를 가진다.
 * 
 * 서비스 혹은 파라미터에 필요 없는 변수들은 단순 무시되고 쓰이지 않는 식이다.
 * 
 * 모두 공통된 모델을 가짐으로써 데이터 관리의 편의성을 도모했다.
 * 
 * 트리의 계층은 
 * root 
 * - CATKIN_WS / ROS 
 * -- Projects
 * --- Services
 * ---- Parameters
 * 이다.
 * 
 */
public class Model_Base {

	public ArrayList<Model_Base> child = new ArrayList<Model_Base>();
	public ArrayList<String> name_cache = new ArrayList<String>();
	public ArrayList<Model_Condition> list_condition = new ArrayList<Model_Condition>();
	public ArrayList<String> headers = new ArrayList<String>();

	public ArrayList<Model_Base> arg_param_list = new ArrayList<Model_Base>();

	public Model_Base parent = null;

	public boolean isRoot = false;
	public boolean isParam = false;
	public boolean isProject = false;
	public boolean isAPI = false;
	public boolean isDefaultParam = false;
	public boolean isArg = false;
	public boolean notFound = false;
	public boolean paramNotOK = false;
	public boolean isIn = false;
	public boolean hasCondition = false;
	public boolean isComboBoxEditorNeeded = false;
	public boolean isConditionSatisfied = false;
	public boolean hasTemplate = false;

	public ParamType ptype;

	public Integer order;
	public Double priority = 5.0;

	public String type = "";
	public String name = "";
	public String tag = "";
	public String value = "";
	public String back_value = "";
	public String location = "";
	public String description = "";
	public String condition_token = "";
	public String operator = "";
	public String[] choices = null;

	public String min = "";
	public String max = "";
	public String not = "";

	private DocumentBuilderFactory factory;
	private DocumentBuilder builder = null;
	private Document doc = null;

	public Model_Base(Model_Base from) {

		if (from.child.size() > 0) {

			for (Model_Base c : from.child) {
				this.child.add(new Model_Base(c));
			}
		}

		this.parent = from.parent;
		this.isRoot = from.isRoot;
		this.isParam = from.isParam;
		this.isProject = from.isProject;
		this.isAPI = from.isAPI;
		this.isDefaultParam = from.isDefaultParam;
		this.isArg = from.isArg;
		this.notFound = from.notFound;
		this.paramNotOK = from.paramNotOK;
		this.isIn = from.isIn;
		this.hasCondition = from.hasCondition;
		this.order = from.order;
		this.priority = from.priority;
		this.type = from.type;
		this.name = from.name;
		this.tag = from.tag;
		this.value = from.value;
		this.location = from.location;
		this.description = from.description;
		this.condition_token = from.condition_token;
		this.operator = from.operator;
		this.choices = from.choices;
	}

	/*
	 * Package 템플릿을 위해서
	 */

	public boolean isTemplate = false;

	public Model_Base(String name, Model_Base parent) {

		if (name.indexOf(".") != -1) {
			name = name.substring(0, name.lastIndexOf("."));
		}

		this.name = name;
		this.parent = parent;
	}

	public Model_Base(String type, String param_name, Model_Base parent) {
		this.type = type;
		this.name = param_name;
		this.parent = parent;
	}

	public Model_Base(boolean isIn, String type, String param_name, Model_Base parent) {
		this.isIn = isIn;
		this.type = type;
		this.name = param_name;
		this.parent = parent;
	}

	public Model_Base(Integer order, String name, Model_Base parent) {
		this.parent = parent;
		this.order = order;
		this.name = name;
	}

	// 사실상 getName과 같은 역할을 한다.
	@Override
	public String toString() {
		String rv = "";
		rv += name;
		return rv;
	}

	public String getValue() {
		return value;
	}

	public void setValue(String s) {
		this.value = s;
	}

	public void setName(String s) {
		this.name = s;
	}

	public Model_Base getParent() {
		return parent;
	}

	public Model_Base getProjectParent() {

		if (this.name.equals("root")) {
			return null;
		}
		if (this.isProject) {
			return this;
		} else {

			return this.parent.getProjectParent();
		}

	}

	public String getType() {
		return type;
	}

	public void setType(String s) {
		this.type = s;
	}

	public String getOrder() {

		return order.toString();
	}

	public void parse_condition(String s) {

		String token = s.substring(0, 1);
		condition_token = token;
		String[] conditions = null;
		String payload = s.substring(1, s.length());
		switch (token) {
		case "1":
			if (payload.indexOf("-%-") != -1) {
				conditions = payload.split("-%-");

				for (String con : conditions) {
					list_condition.add(new Model_Condition(con, this.name, 1));
				}
			} else {
				list_condition.add(new Model_Condition(payload, this.name, 1));
			}
			break;
		case "2":
			String tokens[] = payload.split(",");
			choices = tokens;
			if (choices != null) {
				isComboBoxEditorNeeded = true;
			}
			list_condition.add(new Model_Condition(payload, this.name, 2));

			break;
		default:
			Global.logger("[ERROR] condition parse error");
			break;
		}

		hasCondition = true;

	}

	public class Model_Condition {

		public String condition = "";
		public String expression = "";
		public boolean isSatisfied = false;

		public Model_Condition(String text, String param_name, int token_num) {

			condition = text;

			if (true && false || true) {

			}

			if (token_num == 1) {
				expression += param_name + "  ";

				String[] tokens = text.split(" ");

				if (tokens.length < 2)
					return;

				switch (tokens[0]) {

				case "less_equal":
					expression += "<=  ";
					break;
				case "less":
					expression += "<  ";
					break;
				case "greater_equal":
					expression += ">=  ";
					break;
				case "greater":
					expression += ">  ";
					break;
				case "eqaul":
					expression += "==  ";
					break;
				case "not":
					expression += "!=  ";
					break;
				default:
					expression += "// parsing error //";
					break;

				}

				expression += tokens[1];
			} else if (token_num == 2) {

				expression += "허용 가능 값 : ";

				expression += text;

			}
		}

	}

	public void check_conditions() {

		isConditionSatisfied = true;

		if (isArg) {
			isConditionSatisfied = true;
			return;
		}

		if (ptype.equals(ParamType.Text)) {
			String[] tk = value.split(";");
			if (tk.length < 2) {
				return;
			}
			String tmp = tk[tk.length - 1];
			if (tmp.equals("None")) {
				isConditionSatisfied = false;
				return;
			}

		} else if (ptype.equals(ParamType.Combo)) {
			if (value.length() < 1) {
				isConditionSatisfied = false;
				return;
			}
		} else if (ptype.equals(ParamType.Class)) {
			String[] tk = value.split(";");

			for (String token : tk) {
				if (token.length() < 1)
					continue;
				String[] ttk = token.split(",");
				if (ttk.length < 4)
					continue;
				if (ttk[2].equals("None")) {
					isConditionSatisfied = false;
					return;
				}
			}
		} else if (ptype.equals(ParamType.Vector)) {
			String[] tk = value.split(";");
			String tmp = tk[tk.length - 1];
			if (tmp.equals("None")) {
				isConditionSatisfied = false;
				return;
			}
		}

	}

	public HashMap<String, ArrayList<String>> get_dpendencies() {

		if (isProject == false) {
			return parent.get_dpendencies();
		}

		HashMap<String, ArrayList<String>> hash_depend = new HashMap<String, ArrayList<String>>();
		File file = new File(location + "/package.xml");

		if (file.isFile() == true && file.isDirectory() == false) {

			try {
				factory = DocumentBuilderFactory.newInstance();
				factory.setNamespaceAware(true);

				builder = factory.newDocumentBuilder();
				doc = builder.parse(file);
				Element element_root = doc.getDocumentElement();

				hash_depend.put("depend", get_names(element_root, "depend"));
				hash_depend.put("run_depend", get_names(element_root, "run_depend"));
				hash_depend.put("build_depend", get_names(element_root, "build_depend"));

			} catch (SAXException | IOException | ParserConfigurationException e) {
				e.printStackTrace();
			}

		}

		return hash_depend;

	}

	private ArrayList<String> get_names(Element root, String tagName) {

		ArrayList<String> list = new ArrayList<String>();
		NodeList list_depend = root.getElementsByTagName(tagName);
		boolean isClear = true;
		for (int i = 0; i < list_depend.getLength(); i++) {
			Node node = list_depend.item(i);
			if (node.getNodeType() == Node.ELEMENT_NODE) {
				Element element = (Element) list_depend.item(i);
				String dependName = element.getTextContent().trim();
				if (Global.list_pacakges.contains(dependName) == false) {
					isClear = false;
				}
				list.add(dependName);
			}
		}
		Global.isDependClear = Global.isDependClear && isClear;
		return list;
	}

}
