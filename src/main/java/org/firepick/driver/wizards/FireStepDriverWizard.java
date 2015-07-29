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

package org.firepick.driver.wizards;

import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import org.firepick.driver.FireStepDriver;
import org.openpnp.machine.reference.ReferenceNozzle;
import org.openpnp.machine.reference.driver.wizards.AbstractSerialPortDriverConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.spi.Head;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;

import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.RowSpec;

import javax.swing.JButton;
import javax.swing.JOptionPane;
import javax.swing.JTextField;
import javax.swing.JComboBox;
import javax.swing.DefaultComboBoxModel;

import com.jgoodies.forms.factories.FormFactory;

import javax.swing.JTable;
import javax.swing.JTextPane;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;

public class FireStepDriverWizard  extends AbstractSerialPortDriverConfigurationWizard {
    private final FireStepDriver driver;
    
    public FireStepDriverWizard(FireStepDriver driver) {
        super(driver);
        this.driver = driver;
        
        //Setup panel
        JPanel panelCalibration = new JPanel();
        panelCalibration.setBorder(new TitledBorder(null, "Calibration", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelCalibration);
        contentPanel.add(panelCalibration);
        panelCalibration.setLayout(new FormLayout(new ColumnSpec[] {
        		ColumnSpec.decode("145px:grow"),},
        	new RowSpec[] {
        		RowSpec.decode("23px"),
        		RowSpec.decode("pref:grow"),}));
        
        JButton btnBtnprobe = new JButton("Z Probe");
        btnBtnprobe.addActionListener(new ActionListener() {
        	public void actionPerformed(ActionEvent e) {
        		final Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0); //Assumes one head on the machine
        		try {
        			Location test = new Location(LengthUnit.Millimeters,0,0,0,0);
        			FireStepDriverWizard.this.driver.doZProbePoint((ReferenceNozzle)nozzle, test);
            		//FireStepDriverWizard.this.driver.doDetailedZProbe((ReferenceNozzle)nozzle);
        		}
        		catch (Exception e1){
        			JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
        		}
        	}
        });
        panelCalibration.add(btnBtnprobe, "1, 1, left, top");
        
        JTextPane txtpnProbeResults = new JTextPane();
        txtpnProbeResults.setText("Probe Results");
        panelCalibration.add(txtpnProbeResults, "1, 2, fill, fill");
    }

}
