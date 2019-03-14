package org.etri.eDroneEditor;

import java.util.ArrayList;

public class Model_Section  {

	public int order;
	public String name = "";
	public String origin_name = "";
	public StringBuilder content_stringbuilder = new StringBuilder();
	public String annotation = "";
	public ArrayList<String> list;
	public int count =0 ;
	public Model_Section (String name, String annotation) {
		this.origin_name = name;
		this.name = name;
		this.annotation = annotation;
	}
	public Model_Section (Model_Section model) {
		this.origin_name = model.name;
		this.name = model.name;
		this.annotation = model.annotation;
		this.content_stringbuilder =  new StringBuilder(model.content_stringbuilder);
		this.order = model.order;
	}
	
	public Model_Section (String name, ArrayList <String> list) {
		this.name = name;
		this.list = list;
	}
}
