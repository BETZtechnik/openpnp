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

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JButton;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextPane;
import javax.swing.border.TitledBorder;

import org.firepick.driver.FireStepDriver;
import org.openpnp.machine.reference.driver.wizards.AbstractSerialPortDriverConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.spi.Nozzle;

import com.google.gson.JsonArray;
import com.jgoodies.forms.factories.FormFactory;
import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.RowSpec;

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
                FormFactory.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("default:grow"),},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                RowSpec.decode("default:grow"),}));
        
        JButton btnProbe = new JButton(zProbe);
        panelCalibration.add(btnProbe, "2, 2");
        
        JScrollPane scrollPane = new JScrollPane();
        panelCalibration.add(scrollPane, "2, 4, fill, fill");
        
        textPane = new JTextPane();
        scrollPane.setViewportView(textPane);
    }
 
    @SuppressWarnings("serial")
    private Action zProbe = new AbstractAction("Z Probe") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            final Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0); //Assumes one head on the machine
            try {
                Location test = new Location(LengthUnit.Millimeters,0,0,0,0);
                JsonArray result = driver.doNativeHexZprobe();
                textPane.setText(textPane.getText() + result.toString() + "\r\n");
                //FireStepDriverWizard.this.driver.doDetailedZProbe((ReferenceNozzle)nozzle);
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    private JTextPane textPane;
}
