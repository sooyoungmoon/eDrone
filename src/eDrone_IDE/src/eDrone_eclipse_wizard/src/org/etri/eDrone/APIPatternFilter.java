package org.etri.eDrone;

import org.eclipse.jface.viewers.Viewer;
import org.eclipse.ui.dialogs.PatternFilter;
import org.etri.eDroneModel.Model_Base;

public class APIPatternFilter extends PatternFilter {

	@Override
	public boolean isElementVisible(Viewer viewer, Object element) {

		Model_Base mb = (Model_Base) element;

		return isParentMatch(viewer, element) || isLeafMatch(viewer, element)
				|| isLeafMatch(viewer, mb.parent) && mb.isParam == false;

	}

}