package org.etri.eDrone;

import java.io.File;

import org.eclipse.jface.viewers.ContentViewer;
import org.eclipse.jface.viewers.ILabelProvider;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.ui.dialogs.PatternFilter;

public class FilePatternFilter extends PatternFilter {

	@Override
	protected boolean isLeafMatch(Viewer viewer, Object element) {
		String labelText = ((ILabelProvider) ((ContentViewer) viewer).getLabelProvider()).getText(element);
		if (element instanceof File) {
			File file = (File) element;
			if (isLeafMatch(viewer, file.getParent()))
				return true;
		}
		if (labelText == null) {
			return false;
		}
		return wordMatches(labelText);
	}

}