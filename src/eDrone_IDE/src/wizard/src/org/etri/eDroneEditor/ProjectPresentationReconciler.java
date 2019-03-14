package org.etri.eDroneEditor;

import org.eclipse.jface.text.IDocument;
import org.eclipse.jface.text.TextAttribute;
import org.eclipse.jface.text.presentation.PresentationReconciler;
import org.eclipse.jface.text.rules.DefaultDamagerRepairer;
import org.eclipse.jface.text.rules.IRule;
import org.eclipse.jface.text.rules.RuleBasedScanner;
import org.eclipse.jface.text.rules.SingleLineRule;
import org.eclipse.jface.text.rules.Token;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.RGB;
import org.eclipse.swt.widgets.Display;

public class ProjectPresentationReconciler extends PresentationReconciler {

    private final TextAttribute tagAttribute = new TextAttribute(new Color(Display.getCurrent(), new RGB(0,0, 255)));

    public ProjectPresentationReconciler() {
        RuleBasedScanner scanner= new RuleBasedScanner();
        IRule[] rules = new IRule[1];
        rules[0]= new SingleLineRule("---", "---", new Token(tagAttribute));
        scanner.setRules(rules);
        DefaultDamagerRepairer dr= new DefaultDamagerRepairer(scanner);
        this.setDamager(dr, IDocument.DEFAULT_CONTENT_TYPE);
        this.setRepairer(dr, IDocument.DEFAULT_CONTENT_TYPE);
    }
}