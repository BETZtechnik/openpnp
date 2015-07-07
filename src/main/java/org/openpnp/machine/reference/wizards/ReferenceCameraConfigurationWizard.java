package org.openpnp.machine.reference.wizards;

import java.awt.Color;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.EtchedBorder;
import javax.swing.border.TitledBorder;

import org.jdesktop.beansbinding.AutoBinding.UpdateStrategy;
import org.openpnp.gui.components.ComponentDecorators;
import org.openpnp.gui.support.AbstractConfigurationWizard;
import org.openpnp.gui.support.DoubleConverter;
import org.openpnp.gui.support.LengthConverter;
import org.openpnp.gui.support.MutableLocationProxy;
import org.openpnp.machine.reference.ReferenceCamera;
import org.openpnp.model.Configuration;

import com.jgoodies.forms.factories.FormFactory;
import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.RowSpec;

public class ReferenceCameraConfigurationWizard extends
        AbstractConfigurationWizard {
    private final ReferenceCamera referenceCamera;
    
    private JTextField textFieldOffX;
    private JTextField textFieldOffY;
    private JTextField textFieldOffZ;
    private JPanel panelOff;
    private JPanel panelGeneral;
    private JLabel lblRotation;
    private JTextField textFieldRotation;
    private JLabel lblSafeZ;
    private JTextField textSafeZ;
    
    
    public ReferenceCameraConfigurationWizard(ReferenceCamera referenceCamera) {
        this.referenceCamera = referenceCamera;
        
        panelGeneral = new JPanel();
        panelGeneral.setBorder(new TitledBorder(null, "General", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelGeneral);
        panelGeneral.setLayout(new FormLayout(new ColumnSpec[] {
        		FormFactory.RELATED_GAP_COLSPEC,
        		FormFactory.DEFAULT_COLSPEC,
        		FormFactory.RELATED_GAP_COLSPEC,
        		FormFactory.DEFAULT_COLSPEC,
        		ColumnSpec.decode("default:grow"),},
        	new RowSpec[] {
        		FormFactory.RELATED_GAP_ROWSPEC,
        		FormFactory.DEFAULT_ROWSPEC,
        		FormFactory.RELATED_GAP_ROWSPEC,
        		FormFactory.DEFAULT_ROWSPEC,}));
        
        lblRotation = new JLabel("Rotation");
        panelGeneral.add(lblRotation, "2, 2, right, default");
        
        textFieldRotation = new JTextField();
        panelGeneral.add(textFieldRotation, "4, 2");
        textFieldRotation.setColumns(10);
        
        lblSafeZ = new JLabel("Safe Z");
        panelGeneral.add(lblSafeZ, "2, 4, right, default");
        
        textSafeZ = new JTextField();
        panelGeneral.add(textSafeZ, "4, 4, fill, default");
        textSafeZ.setColumns(10);

        panelOff = new JPanel();
        contentPanel.add(panelOff);
        panelOff.setBorder(new TitledBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null), "Offsets", TitledBorder.LEADING, TitledBorder.TOP, null, new Color(0, 0, 0)));
        panelOff.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));

        JLabel olblX = new JLabel("X");
        panelOff.add(olblX, "2, 2");

        JLabel olblY = new JLabel("Y");
        panelOff.add(olblY, "4, 2");

        JLabel olblZ = new JLabel("Z");
        panelOff.add(olblZ, "6, 2");
        
        
        textFieldOffX = new JTextField();
        panelOff.add(textFieldOffX, "2, 4");
        textFieldOffX.setColumns(8);

        textFieldOffY = new JTextField();
        panelOff.add(textFieldOffY, "4, 4");
        textFieldOffY.setColumns(8);

        textFieldOffZ = new JTextField();
        panelOff.add(textFieldOffZ, "6, 4");
        textFieldOffZ.setColumns(8);
    }
    
    @Override
    public void createBindings() {
        DoubleConverter doubleConverter = new DoubleConverter(Configuration.get().getLengthDisplayFormat());
        LengthConverter lengthConverter = new LengthConverter();
        
        MutableLocationProxy headOffsets = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, referenceCamera, "headOffsets", headOffsets, "location");
        addWrappedBinding(headOffsets, "lengthX", textFieldOffX, "text", lengthConverter);
        addWrappedBinding(headOffsets, "lengthY", textFieldOffY, "text", lengthConverter);
        addWrappedBinding(headOffsets, "lengthZ", textFieldOffZ, "text", lengthConverter);
        addWrappedBinding(referenceCamera, "rotation", textFieldRotation, "text", doubleConverter);
        addWrappedBinding(referenceCamera, "safeZ", textSafeZ, "text", doubleConverter);

        ComponentDecorators.decorateWithAutoSelect(textSafeZ);
        ComponentDecorators.decorateWithAutoSelect(textFieldRotation);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldOffX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldOffY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldOffZ);
    }
}
