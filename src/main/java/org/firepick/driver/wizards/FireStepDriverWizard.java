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

import java.awt.FlowLayout;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.JTextPane;
import javax.swing.border.TitledBorder;

import org.firepick.driver.FireStepDriver;
import org.openpnp.gui.support.IntegerConverter;
import org.openpnp.machine.reference.ReferenceHeadMountable;
import org.openpnp.machine.reference.driver.wizards.AbstractSerialPortDriverConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.model.Location;
import org.openpnp.spi.HeadMountable;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.PasteDispenser;

import com.google.gson.JsonObject;
import com.jgoodies.forms.factories.FormFactory;
import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.RowSpec;

import javax.swing.JCheckBox;

public class FireStepDriverWizard  extends AbstractSerialPortDriverConfigurationWizard {
    private final FireStepDriver driver;
    private List<String> history = new ArrayList<String>();
    private int historyIndex = 0;
    
    public FireStepDriverWizard(FireStepDriver driver) {
        super(driver);
        this.driver = driver;
        
        JPanel panelTools = new JPanel();
        contentPanel.add(panelTools);
        panelTools.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));
        
        chckbxAutoDetectTool = new JCheckBox("Auto Detect Tool Offsets on First Move");
        panelTools.add(chckbxAutoDetectTool, "2, 2");
        
        JPanel panelAngles = new JPanel();
        panelAngles.setBorder(new TitledBorder(null, "Angles", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelAngles);
        panelAngles.setLayout(new FormLayout(new ColumnSpec[] {
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
                FormFactory.DEFAULT_ROWSPEC,}));
        
        JLabel lblX = new JLabel("X");
        panelAngles.add(lblX, "2, 2");
        
        JLabel lblY = new JLabel("Y");
        panelAngles.add(lblY, "4, 2");
        
        JLabel lblZ = new JLabel("Z");
        panelAngles.add(lblZ, "6, 2");
        
        angleX = new JTextField();
        angleX.setText("0");
        panelAngles.add(angleX, "2, 4");
        angleX.setColumns(6);
        angleX.setAction(goToAngles);
        
        angleY = new JTextField();
        angleY.setText("0");
        panelAngles.add(angleY, "4, 4, fill, default");
        angleY.setColumns(6);
        angleY.setAction(goToAngles);
        
        angleZ = new JTextField();
        angleZ.setText("0");
        panelAngles.add(angleZ, "6, 4, fill, default");
        angleZ.setColumns(6);
        angleZ.setAction(goToAngles);
        
        JButton btnGo = new JButton(goToAngles);
        panelAngles.add(btnGo, "8, 4");
        
        JPanel panelTerminal = new JPanel();
        panelTerminal.setBorder(new TitledBorder(null, "Terminal", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelTerminal);
        panelTerminal.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("default:grow"),
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                RowSpec.decode("max(100dlu;default):grow"),}));
        
        terminalCommandTextField = new JTextField();
        panelTerminal.add(terminalCommandTextField, "2, 2, fill, default");
        terminalCommandTextField.setColumns(10);
        terminalCommandTextField.setAction(send);
        terminalCommandTextField.addKeyListener(new KeyAdapter() {
            @Override
            public void keyReleased(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_UP) {
                    if (++historyIndex >= history.size()) {
                        historyIndex = 0;
                    }
                    terminalCommandTextField.setText(history.get(historyIndex));
                }
                else if (e.getKeyCode() == KeyEvent.VK_DOWN) {
                    if (--historyIndex < 0) {
                        historyIndex = history.size() - 1;
                    }
                    terminalCommandTextField.setText(history.get(historyIndex));
                }
                else {
                    super.keyTyped(e);
                }
            }
        });
        
        JButton btnSend = new JButton(send);
        panelTerminal.add(btnSend, "4, 2");
        
        JScrollPane scrollPane_1 = new JScrollPane();
        panelTerminal.add(scrollPane_1, "2, 4, 3, 1, fill, fill");
        
        terminalLogTextPane = new JTextPane();
        scrollPane_1.setViewportView(terminalLogTextPane);
        
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
                RowSpec.decode("max(100px;default):grow"),}));
        
        JPanel panel = new JPanel();
        FlowLayout flowLayout = (FlowLayout) panel.getLayout();
        flowLayout.setAlignment(FlowLayout.LEFT);
        panelCalibration.add(panel, "2, 2, fill, fill");
        
        JButton singleZprobeButton = new JButton(singleZprobe);
        panel.add(singleZprobeButton);
        
        JButton btnHexZProbe = new JButton(hexZprobe);
        panel.add(btnHexZProbe);
        
        JButton btnDetailedZProbe = new JButton(detailedZprobe);
        panel.add(btnDetailedZProbe);
        
        JButton btnGfilter = new JButton(gFilter);
        panel.add(btnGfilter);
        
        JScrollPane scrollPane = new JScrollPane();
        panelCalibration.add(scrollPane, "2, 4, fill, fill");
        
        zProbeResultsTextPane = new JTextPane();
        scrollPane.setViewportView(zProbeResultsTextPane);
        
        JPanel panelCalibration2 = new JPanel();
        contentPanel.add(panelCalibration2);
        panelCalibration2.setLayout(new FormLayout(new ColumnSpec[] {
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
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));
        
        JButton btnCornersZProbe = new JButton(cornerZprobe);
        panelCalibration2.add(btnCornersZProbe, "4, 2");
        
        labelBackLeft = new JLabel("100.000");
        panelCalibration2.add(labelBackLeft, "2, 4");
        
        labelBackRight = new JLabel("100.000");
        panelCalibration2.add(labelBackRight, "6, 4");
        
        labelFrontLeft = new JLabel("100.000");
        panelCalibration2.add(labelFrontLeft, "2, 8");
        
        labelFrontRight = new JLabel("100.000");
        panelCalibration2.add(labelFrontRight, "6, 8");
    }
 
    @Override
    public void createBindings() {
        super.createBindings();
        addWrappedBinding(driver, "autoUpdateToolOffsets", chckbxAutoDetectTool, "selected");
    }

    @SuppressWarnings("serial")
    private Action singleZprobe = new AbstractAction("Single Z Probe") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            final Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0); //Assumes one head on the machine
            try {
                Location originalLocation = nozzle.getLocation();
                Location result = driver.doZprobePoint((ReferenceHeadMountable) nozzle, nozzle.getLocation());
                nozzle.moveTo(originalLocation, 1.0);
                zProbeResultsTextPane.setText(zProbeResultsTextPane.getText() + result.toString() + "\r\n");
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    @SuppressWarnings("serial")
    private Action hexZprobe = new AbstractAction("Hex Z Probe") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            final Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0); //Assumes one head on the machine
            try {
                List<Location> results = driver.doZprobeHex((ReferenceHeadMountable) nozzle);
                zProbeResultsTextPane.setText(zProbeResultsTextPane.getText() + results.toString() + "\r\n");
                terminalLogTextPane.scrollRectToVisible(new Rectangle(0, terminalLogTextPane.getBounds(null).height, 1, 1));            
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    @SuppressWarnings("serial")
    private Action detailedZprobe = new AbstractAction("Detailed Z Probe") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            final Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0); //Assumes one head on the machine
            try {
                double[][] results = driver.doZProbeDetailed((ReferenceHeadMountable) nozzle);
                resolveMissingDetailedZprobeData(results);
                String s = "";
                for (int y = 0, yCount = results.length; y < yCount; y++) {
                    for (int x = 0, xCount = results.length; x < xCount; x++) {
                        s += results[x][y];
                        if (x < xCount - 1) {
                            s += "\t";
                        }
                    }
                    s += "\r\n";
                }
                zProbeResultsTextPane.setText(zProbeResultsTextPane.getText() + s);
                terminalLogTextPane.scrollRectToVisible(new Rectangle(0, terminalLogTextPane.getBounds(null).height, 1, 1));            
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
                e1.printStackTrace();
            }
        }
    };
    
    static private void resolveMissingDetailedZprobeData(double[][] values) {
        boolean done = true;
        for (int y = 0; y < values.length; y++) {
            for (int x = 0; x < values.length; x++) {
                if (Double.isNaN(values[x][y])) {
                    values[x][y] = getNeighborAverage(values, x, y);
                    if (Double.isNaN(values[x][y])) {
                        done = false;
                    }
                }
            }
        }
        if (!done) {
            resolveMissingDetailedZprobeData(values);
        }
    }
    
    /**
     * Calculate the average values of the neighbors of a point in a 2D array.
     * Array must be square.
     * @param values
     * @param x
     * @param y
     * @return
     */
    static private double getNeighborAverage(double[][] values, int x, int y) {
        double sum = 0;
        int count = 0;
        for (int xx = -1; xx <= 1; xx++) {
            for (int yy = -1; yy <= 1; yy++) {
                if (xx == 0 && yy == 0) {
                    continue;
                }
                if (x + xx >= 0 && y + yy >= 0 && x + xx < values.length && y + yy < values.length) {
                    if (!Double.isNaN(values[x + xx][y + yy])) {
                        sum += values[x + xx][y + yy];
                        count++;
                    }
                }
            }
        }
        return sum / count;
    }
    
    @SuppressWarnings("serial")
    private Action cornerZprobe = new AbstractAction("Corner Z Probe") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            final Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0); //Assumes one head on the machine
            try {
                List<Location> results = driver.doZprobeCorners((ReferenceHeadMountable) nozzle);
                // front left, front right, back right, back left
                labelFrontLeft.setText("" + results.get(0).getZ());
                labelFrontRight.setText("" + results.get(1).getZ());
                labelBackRight.setText("" + results.get(2).getZ());
                labelBackLeft.setText("" + results.get(3).getZ());
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    
    @SuppressWarnings("serial")
    private Action send = new AbstractAction("Send") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                List<JsonObject> responses = driver.sendJsonCommand(terminalCommandTextField.getText(), 30000);
                String s = "";
                for (JsonObject o : responses) {
                    s += o.toString() + "\r\n";
                }
                terminalLogTextPane.setText(terminalLogTextPane.getText() + s);
                terminalLogTextPane.scrollRectToVisible(new Rectangle(0, terminalLogTextPane.getBounds(null).height, 1, 1));
                history.add(0, terminalCommandTextField.getText());
                historyIndex = 0;
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    @SuppressWarnings("serial")
    private Action goToAngles = new AbstractAction("Go") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                driver.moveToAngles(Double.parseDouble(angleX.getText()), Double.parseDouble(angleY.getText()), Double.parseDouble(angleZ.getText()));
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    @SuppressWarnings("serial")
    private Action gFilter = new AbstractAction("G-Filter") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                driver.generateGfilter();
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    @SuppressWarnings("serial")
    private Action detectTool = new AbstractAction("Detect Tool") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                HeadMountable hm = driver.detectInstalledTool();
                System.out.println(hm);
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    @SuppressWarnings("serial")
    private Action checkToolOffsets = new AbstractAction("Check Offsets") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                driver.checkToolOffsets();
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    private JTextField terminalCommandTextField;
    private JTextPane terminalLogTextPane;
    private JTextPane zProbeResultsTextPane;
    private JLabel labelFrontRight;
    private JLabel labelFrontLeft;
    private JLabel labelBackLeft;
    private JLabel labelBackRight;
    private JTextField angleX;
    private JTextField angleY;
    private JTextField angleZ;
    private JCheckBox chckbxAutoDetectTool;
}
