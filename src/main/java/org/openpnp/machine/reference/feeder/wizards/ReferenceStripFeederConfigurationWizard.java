/*
 	Copyright (C) 2011 Jason von Nieda <jason@vonnieda.org>
 	
 	This file is part of OpenPnP.
 	
	OpenPnP is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenPnP is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with OpenPnP.  If not, see <http://www.gnu.org/licenses/>.
 	
 	For more information about OpenPnP visit http://openpnp.org
 */

package org.openpnp.machine.reference.feeder.wizards;

import javax.swing.AbstractAction;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.TitledBorder;

import org.jdesktop.beansbinding.AutoBinding.UpdateStrategy;
import org.openpnp.gui.components.ComponentDecorators;
import org.openpnp.gui.components.LocationButtonsPanel;
import org.openpnp.gui.support.BufferedImageIconConverter;
import org.openpnp.gui.support.DoubleConverter;
import org.openpnp.gui.support.IntegerConverter;
import org.openpnp.gui.support.LengthConverter;
import org.openpnp.gui.support.MutableLocationProxy;
import org.openpnp.machine.reference.feeder.ReferenceStripFeeder;
import org.openpnp.model.Configuration;

import com.jgoodies.forms.factories.FormFactory;
import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.RowSpec;

import javax.swing.border.EtchedBorder;

import java.awt.Color;
import java.awt.event.ActionEvent;

import javax.swing.JButton;

@SuppressWarnings("serial")
public class ReferenceStripFeederConfigurationWizard extends
        AbstractReferenceFeederConfigurationWizard {
    private final ReferenceStripFeeder feeder;

    private JTextField textFieldFeedStartX;
    private JTextField textFieldFeedStartY;
    private JTextField textFieldFeedStartZ;
    private JTextField textFieldFeedEndX;
    private JTextField textFieldFeedEndY;
    private JTextField textFieldFeedEndZ;
    private JTextField textFieldTapeWidth;
    private JLabel lblPartPitch;
    private JTextField textFieldPartPitch;
    private JPanel panelTapeSettings;
    private JPanel panelLocations;
    private LocationButtonsPanel locationButtonsPanelFeedStart;
    private LocationButtonsPanel locationButtonsPanelFeedEnd;
    private JPanel panelGeneralSettings;
    private JLabel lblFeedCount;
    private JTextField textFieldFeedCount;
    private JButton btnResetFeedCount;

    public ReferenceStripFeederConfigurationWizard(ReferenceStripFeeder feeder) {
        super(feeder, false);
        this.feeder = feeder;

        JPanel panelFields = new JPanel();
        panelFields.setLayout(new BoxLayout(panelFields, BoxLayout.Y_AXIS));

        contentPanel.add(panelFields);

        panelLocations = new JPanel();
        panelFields.add(panelLocations);
        panelLocations.setBorder(new TitledBorder(null, "Locations",
                TitledBorder.LEADING, TitledBorder.TOP, null, null));
        panelLocations
                .setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("left:default:grow"),},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));

        JLabel lblX = new JLabel("X");
        panelLocations.add(lblX, "4, 2");

        JLabel lblY = new JLabel("Y");
        panelLocations.add(lblY, "6, 2");

        JLabel lblZ = new JLabel("Z");
        panelLocations.add(lblZ, "8, 2");

        JLabel lblFeedStartLocation = new JLabel("Reference Hole Location");
        panelLocations.add(lblFeedStartLocation, "2, 4, right, default");

        textFieldFeedStartX = new JTextField();
        panelLocations.add(textFieldFeedStartX, "4, 4");
        textFieldFeedStartX.setColumns(8);

        textFieldFeedStartY = new JTextField();
        panelLocations.add(textFieldFeedStartY, "6, 4");
        textFieldFeedStartY.setColumns(8);

        textFieldFeedStartZ = new JTextField();
        panelLocations.add(textFieldFeedStartZ, "8, 4");
        textFieldFeedStartZ.setColumns(8);

        locationButtonsPanelFeedStart = new LocationButtonsPanel(
                textFieldFeedStartX, textFieldFeedStartY, textFieldFeedStartZ,
                null);
        panelLocations.add(locationButtonsPanelFeedStart, "10, 4");

        JLabel lblFeedEndLocation = new JLabel("Last Hole Location");
        panelLocations.add(lblFeedEndLocation, "2, 6, right, default");

        textFieldFeedEndX = new JTextField();
        panelLocations.add(textFieldFeedEndX, "4, 6");
        textFieldFeedEndX.setColumns(8);

        textFieldFeedEndY = new JTextField();
        panelLocations.add(textFieldFeedEndY, "6, 6");
        textFieldFeedEndY.setColumns(8);

        textFieldFeedEndZ = new JTextField();
        panelLocations.add(textFieldFeedEndZ, "8, 6");
        textFieldFeedEndZ.setColumns(8);

        locationButtonsPanelFeedEnd = new LocationButtonsPanel(
                textFieldFeedEndX, textFieldFeedEndY, textFieldFeedEndZ, null);
        panelLocations.add(locationButtonsPanelFeedEnd, "10, 6");

        panelTapeSettings = new JPanel();
        panelTapeSettings.setBorder(new TitledBorder(new EtchedBorder(
                EtchedBorder.LOWERED, null, null), "Tape Settings",
                TitledBorder.LEADING, TitledBorder.TOP, null,
                new Color(0, 0, 0)));

        panelFields.add(panelTapeSettings);
        panelTapeSettings.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));

        JLabel lblTapeWidth = new JLabel("Tape Width");
        panelTapeSettings.add(lblTapeWidth, "2, 2");

        textFieldTapeWidth = new JTextField();
        panelTapeSettings.add(textFieldTapeWidth, "4, 2");
        textFieldTapeWidth.setColumns(5);

        lblPartPitch = new JLabel("Part Pitch");
        panelTapeSettings.add(lblPartPitch, "2, 4, right, default");

        textFieldPartPitch = new JTextField();
        panelTapeSettings.add(textFieldPartPitch, "4, 4");
        textFieldPartPitch.setColumns(5);
        
        panelGeneralSettings = new JPanel();
        panelGeneralSettings.setBorder(new TitledBorder(null, "General Settings", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        panelFields.add(panelGeneralSettings);
        panelGeneralSettings.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));
        
        lblFeedCount = new JLabel("Feed Count");
        panelGeneralSettings.add(lblFeedCount, "2, 2, right, default");
        
        textFieldFeedCount = new JTextField();
        panelGeneralSettings.add(textFieldFeedCount, "4, 2, fill, default");
        textFieldFeedCount.setColumns(10);
        
        btnResetFeedCount = new JButton(new AbstractAction("Reset") {
            @Override
            public void actionPerformed(ActionEvent e) {
                textFieldFeedCount.setText("0");
                applyAction.actionPerformed(e);
            }
        });
        panelGeneralSettings.add(btnResetFeedCount, "6, 2");
    }

    @Override
    public void createBindings() {
        super.createBindings();
        LengthConverter lengthConverter = new LengthConverter();
        IntegerConverter intConverter = new IntegerConverter();
        DoubleConverter doubleConverter = new DoubleConverter(Configuration
                .get().getLengthDisplayFormat());
        BufferedImageIconConverter imageConverter = new BufferedImageIconConverter();

        addWrappedBinding(feeder, "tapeWidth", textFieldTapeWidth, "text",
                lengthConverter);
        addWrappedBinding(feeder, "partPitch", textFieldPartPitch, "text",
                lengthConverter);
        addWrappedBinding(feeder, "feedCount", textFieldFeedCount, "text",
                intConverter);

        MutableLocationProxy feedStartLocation = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "referenceHoleLocation",
                feedStartLocation, "location");
        addWrappedBinding(feedStartLocation, "lengthX", textFieldFeedStartX,
                "text", lengthConverter);
        addWrappedBinding(feedStartLocation, "lengthY", textFieldFeedStartY,
                "text", lengthConverter);
        addWrappedBinding(feedStartLocation, "lengthZ", textFieldFeedStartZ,
                "text", lengthConverter);

        MutableLocationProxy feedEndLocation = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "lastHoleLocation",
                feedEndLocation, "location");
        addWrappedBinding(feedEndLocation, "lengthX", textFieldFeedEndX,
                "text", lengthConverter);
        addWrappedBinding(feedEndLocation, "lengthY", textFieldFeedEndY,
                "text", lengthConverter);
        addWrappedBinding(feedEndLocation, "lengthZ", textFieldFeedEndZ,
                "text", lengthConverter);

        ComponentDecorators
                .decorateWithAutoSelectAndLengthConversion(textFieldTapeWidth);
        ComponentDecorators
                .decorateWithAutoSelectAndLengthConversion(textFieldPartPitch);
        ComponentDecorators
                .decorateWithAutoSelect(textFieldFeedCount);
        ComponentDecorators
                .decorateWithAutoSelectAndLengthConversion(textFieldFeedStartX);
        ComponentDecorators
                .decorateWithAutoSelectAndLengthConversion(textFieldFeedStartY);
        ComponentDecorators
                .decorateWithAutoSelectAndLengthConversion(textFieldFeedStartZ);
        ComponentDecorators
                .decorateWithAutoSelectAndLengthConversion(textFieldFeedEndX);
        ComponentDecorators
                .decorateWithAutoSelectAndLengthConversion(textFieldFeedEndY);
        ComponentDecorators
                .decorateWithAutoSelectAndLengthConversion(textFieldFeedEndZ);
    }
}