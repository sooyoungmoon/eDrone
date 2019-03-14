package org.etri.eDroneModel;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

public class Model_Service {
	public boolean hasContents = true;
	public File file;
	public String file_path = "";
	public ArrayList<Model_Base> api_list = new ArrayList<Model_Base>();
	public HashMap<String, Integer> hash_hierarchy = new HashMap<String, Integer>();
	public ArrayList<Model_Base> depend_list = new ArrayList<Model_Base>();
	public String name = "";
	public String description = "";
}
