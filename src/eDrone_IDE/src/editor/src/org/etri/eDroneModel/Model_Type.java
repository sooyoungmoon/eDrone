package org.etri.eDroneModel;

import java.util.HashMap;

import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class Model_Type {

	public String name = "";
	public String beConverted = "";
	public HashMap<String, Model_Type> child = new HashMap<String, Model_Type>();

	public Model_Type(Element e) {

		name = e.getAttribute("from");
		beConverted = e.getAttribute("to");

		NodeList list_basicType = e.getChildNodes();

		for (int i = 0; i < list_basicType.getLength(); i++) {
			Node node = list_basicType.item(i);
			if (node.getNodeType() == Node.ELEMENT_NODE) {
				Element element = (Element) node;
				child.put(element.getAttribute("from"), new Model_Type(element));
			}
		}
	}
}
