package org.etri.eDroneEditor;

import java.io.File;
import java.util.ArrayList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.eclipse.jface.text.BadLocationException;
import org.eclipse.jface.text.IDocument;
import org.eclipse.jface.text.ITextViewer;
import org.eclipse.jface.text.contentassist.CompletionProposal;
import org.eclipse.jface.text.contentassist.ICompletionProposal;
import org.eclipse.jface.text.contentassist.IContentAssistProcessor;
import org.eclipse.jface.text.contentassist.IContextInformation;
import org.eclipse.jface.text.contentassist.IContextInformationValidator;
import org.eclipse.swt.graphics.Image;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class ProjectContentAssistProcessor implements IContentAssistProcessor {

	ArrayList<Model_Section> list_sections;

	@Override
	public ICompletionProposal[] computeCompletionProposals(ITextViewer viewer, int offset) {

		String text = "";
		try {

			if (offset < 1)
				return new ICompletionProposal[0];

			int s_index = offset - 1;

			IDocument id = viewer.getDocument();
			int length = 0;
			String token;
			String cache = "";
			int cache_index = -1;
			int cache_length = -1;
			boolean cacheOpen = false;
			while (s_index > 0) {

				token = id.get(s_index, 1);

				if (token.equals("-")) {
					if (cacheOpen == false) {
						cache_index = s_index + 1;
						cache_length = length;
						cacheOpen = true;
						cache += token;
					} else {
						cache += token;
					}
				} else if (token.equals("\n")) {
					if (cacheOpen == true) {
						cacheOpen = false;
						if (cache.equals("---")) {
							s_index = cache_index;
							length = cache_length;
							break;
						}
					}
					break;
				}

				s_index--;
				length++;
			}

			text = id.get(s_index, length + 1);

			text = text.trim();

			if (text.indexOf(",") != -1) {
				String tokens[] = text.split(",");

				text = tokens[0];

			}

		} catch (BadLocationException e) {
			e.printStackTrace();
		}

//		System.out.println(text);

		Image image_logo = Activator.getImageDescriptor("icons/logo_@1x.png").createImage();

		ArrayList<ICompletionProposal> proposal_list = new ArrayList<ICompletionProposal>();
		String workspaceLoc = System.getenv("CATKIN_WS_PATH");
		read_section(new File(workspaceLoc + "/edrone_mc_support/template/source/section.xml"));

		for (Model_Section ms : list_sections) {
			System.out.println(ms.name);
			if (ms.name.startsWith(text)) {

				String name = ms.name;
				if (name.length() < 1 || name == null)
					continue;

				String part = name.substring(text.length(), name.length());

				proposal_list.add(new CompletionProposal(part + "\n---", offset, 0, part.length(), image_logo, name,
						null, ms.annotation));

			}
		}
		ICompletionProposal[] array = new ICompletionProposal[proposal_list.size()];
		int i = 0;
		for (ICompletionProposal icp : proposal_list) {
			array[i] = icp;
			i++;
		}

		return array;
	}

	public void read_section(File f) {

		DocumentBuilderFactory factory;
		DocumentBuilder builder = null;
		Document doc = null;

		list_sections = new ArrayList<Model_Section>();

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
					list_sections.add(model_sec);
				}
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public IContextInformation[] computeContextInformation(ITextViewer viewer, int offset) {
		return null;
	}

	@Override
	public char[] getCompletionProposalAutoActivationCharacters() {
		return new char[] { '"' }; // NON-NLS-1
	}

	@Override
	public char[] getContextInformationAutoActivationCharacters() {
		return null;
	}

	@Override
	public String getErrorMessage() {
		return null;
	}

	@Override
	public IContextInformationValidator getContextInformationValidator() {
		return null;
	}

}