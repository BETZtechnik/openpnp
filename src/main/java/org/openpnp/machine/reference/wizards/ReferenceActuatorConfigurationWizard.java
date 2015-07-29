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
import org.openpnp.machine.reference.ReferenceActuator;
import org.openpnp.model.Configuration;

import com.jgoodies.forms.factories.FormFactory;
import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.RowSpec;

import java.awt.Component;

public class ReferenceActuatorConfigurationWizard extends
		AbstractConfigurationWizard {
	private final ReferenceActuator actuator;

	private JTextField locationX;
	private JTextField locationY;
	private JTextField locationZ;
	private JPanel panelOffsets;
	private JPanel panelSafeZ;
	private JLabel lblSafeZ;
	private JTextField textSafeZ;

	public ReferenceActuatorConfigurationWizard(ReferenceActuator actuator) {
		this.actuator = actuator;

		panelOffsets = new JPanel();
		panelOffsets.setBorder(new TitledBorder(new EtchedBorder(
				EtchedBorder.LOWERED, null, null), "Offsets",
				TitledBorder.LEADING, TitledBorder.TOP, null,
				new Color(0, 0, 0)));
		panelOffsets.setLayout(new FormLayout(
				new ColumnSpec[] { FormFactory.RELATED_GAP_COLSPEC,
						FormFactory.DEFAULT_COLSPEC,
						FormFactory.RELATED_GAP_COLSPEC,
						FormFactory.DEFAULT_COLSPEC,
						FormFactory.RELATED_GAP_COLSPEC,
						FormFactory.DEFAULT_COLSPEC,
						FormFactory.RELATED_GAP_COLSPEC,
						FormFactory.DEFAULT_COLSPEC, }, new RowSpec[] {
						FormFactory.RELATED_GAP_ROWSPEC,
						FormFactory.DEFAULT_ROWSPEC,
						FormFactory.RELATED_GAP_ROWSPEC,
						FormFactory.DEFAULT_ROWSPEC, }));

		JLabel lblX = new JLabel("X");
		panelOffsets.add(lblX, "2, 2");

		JLabel lblY = new JLabel("Y");
		panelOffsets.add(lblY, "4, 2");

		JLabel lblZ = new JLabel("Z");
		panelOffsets.add(lblZ, "6, 2");

		locationX = new JTextField();
		panelOffsets.add(locationX, "2, 4");
		locationX.setColumns(5);

		locationY = new JTextField();
		panelOffsets.add(locationY, "4, 4");
		locationY.setColumns(5);

		locationZ = new JTextField();
		panelOffsets.add(locationZ, "6, 4");
		locationZ.setColumns(5);

		contentPanel.add(panelOffsets);

		panelSafeZ = new JPanel();
		panelSafeZ.setBorder(new TitledBorder(new EtchedBorder(
				EtchedBorder.LOWERED, null, null), "Safe Z",
				TitledBorder.LEADING, TitledBorder.TOP, null,
				new Color(0, 0, 0)));
		contentPanel.add(panelSafeZ);
		panelSafeZ.setLayout(new FormLayout(new ColumnSpec[] {
				FormFactory.RELATED_GAP_COLSPEC, ColumnSpec.decode("38px"),
				FormFactory.LABEL_COMPONENT_GAP_COLSPEC,
				ColumnSpec.decode("134px"), }, new RowSpec[] {
				RowSpec.decode("23px"), RowSpec.decode("28px"), }));

		lblSafeZ = new JLabel("Safe Z");
		panelSafeZ.add(lblSafeZ, "2, 2, left, center");

		textSafeZ = new JTextField();
		panelSafeZ.add(textSafeZ, "4, 2, left, top");
		textSafeZ.setColumns(10);
	}

	@Override
	public void createBindings() {
        DoubleConverter doubleConverter = new DoubleConverter(Configuration.get().getLengthDisplayFormat());
		LengthConverter lengthConverter = new LengthConverter();

		MutableLocationProxy headOffsets = new MutableLocationProxy();
		bind(UpdateStrategy.READ_WRITE, actuator, "headOffsets", headOffsets,
				"location");
		addWrappedBinding(headOffsets, "lengthX", locationX, "text",
				lengthConverter);
		addWrappedBinding(headOffsets, "lengthY", locationY, "text",
				lengthConverter);
		addWrappedBinding(headOffsets, "lengthZ", locationZ, "text",
				lengthConverter);

        addWrappedBinding(actuator, "safeZ", textSafeZ, "text", doubleConverter);

        ComponentDecorators.decorateWithAutoSelect(textSafeZ);

        ComponentDecorators
				.decorateWithAutoSelectAndLengthConversion(locationX);
		ComponentDecorators
				.decorateWithAutoSelectAndLengthConversion(locationY);
		ComponentDecorators
				.decorateWithAutoSelectAndLengthConversion(locationZ);
	}
}