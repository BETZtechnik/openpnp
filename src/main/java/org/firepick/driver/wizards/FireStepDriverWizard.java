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

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Future;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.JTextPane;
import javax.swing.SwingUtilities;
import javax.swing.border.EtchedBorder;
import javax.swing.border.TitledBorder;

import org.firepick.driver.FireStepDriver;
import org.firepick.kinematics.RotatableDeltaKinematicsCalculator;
import org.jdesktop.beansbinding.AutoBinding.UpdateStrategy;
import org.openpnp.gui.components.ComponentDecorators;
import org.openpnp.gui.components.LocationButtonsPanel;
import org.openpnp.gui.support.DoubleConverter;
import org.openpnp.gui.support.IntegerConverter;
import org.openpnp.gui.support.LengthConverter;
import org.openpnp.gui.support.MessageBoxes;
import org.openpnp.gui.support.MutableLocationProxy;
import org.openpnp.machine.reference.ReferenceHeadMountable;
import org.openpnp.machine.reference.driver.wizards.AbstractSerialPortDriverConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.model.Location;
import org.openpnp.spi.Nozzle;

import com.google.common.util.concurrent.FutureCallback;
import com.google.gson.JsonObject;
import com.jgoodies.forms.factories.FormFactory;
import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.RowSpec;

public class FireStepDriverWizard  extends AbstractSerialPortDriverConfigurationWizard {
    private final FireStepDriver driver;
    private List<String> history = new ArrayList<String>();
    private int historyIndex = 0;
    
    public FireStepDriverWizard(FireStepDriver driver) {
        super(driver);
        this.driver = driver;
        
        JPanel panelDelta = new JPanel();
        panelDelta.setBorder(new TitledBorder(null, "Delta Settings", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelDelta);
        panelDelta.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
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
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));
        
        JLabel lblUseFirestepKinematics = new JLabel("Use FireStep Kinematics?");
        panelDelta.add(lblUseFirestepKinematics, "2, 2");
        
        checkBoxFireStepKinematics = new JCheckBox("");
        panelDelta.add(checkBoxFireStepKinematics, "4, 2");
        
        JLabel lblEndEffectorArm = new JLabel("End Effector Arm Length");
        panelDelta.add(lblEndEffectorArm, "2, 6");
        
        JLabel lblServoHornLength = new JLabel("Servo Horn Length");
        panelDelta.add(lblServoHornLength, "4, 6");
        
        JLabel lblEndEffectorLength = new JLabel("End Effector Length");
        panelDelta.add(lblEndEffectorLength, "6, 6");
        
        JLabel lblBaseLength = new JLabel("Base Length");
        panelDelta.add(lblBaseLength, "8, 6");
        
        textFieldRe = new JTextField();
        panelDelta.add(textFieldRe, "2, 8, fill, default");
        textFieldRe.setColumns(10);
        
        textFieldRf = new JTextField();
        panelDelta.add(textFieldRf, "4, 8, fill, default");
        textFieldRf.setColumns(10);
        
        textFieldE = new JTextField();
        panelDelta.add(textFieldE, "6, 8, fill, default");
        textFieldE.setColumns(10);
        
        textFieldF = new JTextField();
        panelDelta.add(textFieldF, "8, 8, fill, default");
        textFieldF.setColumns(10);
        
        JLabel lblMotorStepsPer = new JLabel("Motor Steps Per Rotation");
        panelDelta.add(lblMotorStepsPer, "2, 10");
        
        JLabel lblMotorMicrostepsPer = new JLabel("Motor Microsteps Per Step");
        panelDelta.add(lblMotorMicrostepsPer, "4, 10");
        
        textFieldSteps = new JTextField();
        panelDelta.add(textFieldSteps, "2, 12, fill, default");
        textFieldSteps.setColumns(10);
        
        textFieldMicrosteps = new JTextField();
        panelDelta.add(textFieldMicrosteps, "4, 12, fill, default");
        textFieldMicrosteps.setColumns(10);
        
        JLabel lblX = new JLabel("X");
        panelDelta.add(lblX, "4, 16, center, default");
        
        JLabel lblY = new JLabel("Y");
        panelDelta.add(lblY, "6, 16, center, default");
        
        JLabel lblZ = new JLabel("Z");
        panelDelta.add(lblZ, "8, 16, center, default");
        
        JLabel lblHomeAngleStep = new JLabel("Home Angle Step Counts");
        panelDelta.add(lblHomeAngleStep, "2, 18");
        
        textFieldHascX = new JTextField();
        panelDelta.add(textFieldHascX, "4, 18, fill, default");
        textFieldHascX.setColumns(10);
        
        textFieldHascY = new JTextField();
        panelDelta.add(textFieldHascY, "6, 18, fill, default");
        textFieldHascY.setColumns(10);
        
        textFieldHascZ = new JTextField();
        panelDelta.add(textFieldHascZ, "8, 18, fill, default");
        textFieldHascZ.setColumns(10);
        
        JButton btnReadHasc = new JButton(hascRead);
        panelDelta.add(btnReadHasc, "10, 18");
        
        JLabel lblGearRatios = new JLabel("Gear Ratios");
        panelDelta.add(lblGearRatios, "2, 20, right, default");
        
        textFieldGrX = new JTextField();
        panelDelta.add(textFieldGrX, "4, 20, fill, default");
        textFieldGrX.setColumns(10);
        
        textFieldGrY = new JTextField();
        panelDelta.add(textFieldGrY, "6, 20, fill, default");
        textFieldGrY.setColumns(10);
        
        textFieldGrZ = new JTextField();
        panelDelta.add(textFieldGrZ, "8, 20, fill, default");
        textFieldGrZ.setColumns(10);
        
        JPanel panelBedLeveling = new JPanel();
        panelBedLeveling.setBorder(new TitledBorder(null, "Bed Leveling", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelBedLeveling);
        panelBedLeveling.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("default:grow"),},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                RowSpec.decode("default:grow"),
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));
        
        JButton btnCornersZProbe = new JButton(probeCorners);
        panelBedLeveling.add(btnCornersZProbe, "4, 2");
        
        JButton singleZprobeButton = new JButton(probePoint);
        panelBedLeveling.add(singleZprobeButton, "10, 2");
        
        JButton btnDetailedZProbe = new JButton(probeGrid);
        panelBedLeveling.add(btnDetailedZProbe, "14, 2, center, default");
        
        labelBackLeft = new JLabel("100.000");
        panelBedLeveling.add(labelBackLeft, "2, 4");
        
        labelBackRight = new JLabel("100.000");
        panelBedLeveling.add(labelBackRight, "6, 4");
        
        labelSingleProbeResults = new JLabel("100.000");
        panelBedLeveling.add(labelSingleProbeResults, "10, 4, center, default");
        
        JScrollPane scrollPane_2 = new JScrollPane();
        panelBedLeveling.add(scrollPane_2, "14, 4, 1, 5, fill, fill");
        
        textPaneZprobeDetailedResults = new JTextPane();
        scrollPane_2.setViewportView(textPaneZprobeDetailedResults);
        
        labelFrontLeft = new JLabel("100.000");
        panelBedLeveling.add(labelFrontLeft, "2, 8");
        
        labelFrontRight = new JLabel("100.000");
        panelBedLeveling.add(labelFrontRight, "6, 8");
        
        //Setup panel
        JPanel panelBarycentricCalibration = new JPanel();
        panelBarycentricCalibration.setBorder(new TitledBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null), "Barycentric Calibration", TitledBorder.LEADING, TitledBorder.TOP, null, new Color(0, 0, 0)));
        contentPanel.add(panelBarycentricCalibration);
        contentPanel.add(panelBarycentricCalibration);
        panelBarycentricCalibration.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));
        
        chckbxEnableBarycentricInterpolation = new JCheckBox("Enable Barycentric Interpolation?");
        panelBarycentricCalibration.add(chckbxEnableBarycentricInterpolation, "2, 1");
        
        JButton btnGfilter = new JButton(barycentricCaptureFull);
        panelBarycentricCalibration.add(btnGfilter, "2, 3");
        
        JButton btnNewButton = new JButton(barycentricCaptureUnmapped);
        panelBarycentricCalibration.add(btnNewButton, "4, 3");
        
        JPanel panelCameraPose = new JPanel();
        panelCameraPose.setBorder(new TitledBorder(null, "Camera Pose Calibration", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelCameraPose);
        panelCameraPose.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,
                FormFactory.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("default:grow"),},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,
                FormFactory.RELATED_GAP_ROWSPEC,
                RowSpec.decode("default:grow"),
                FormFactory.RELATED_GAP_ROWSPEC,
                RowSpec.decode("default:grow"),}));
        
        chckbxEnableCameraPose = new JCheckBox("Enable Camera Pose Interpolation?");
        panelCameraPose.add(chckbxEnableCameraPose, "2, 2, 7, 1");
        
        JLabel lblX_1 = new JLabel("X");
        panelCameraPose.add(lblX_1, "4, 4");
        
        JLabel lblY_1 = new JLabel("Y");
        panelCameraPose.add(lblY_1, "6, 4");
        
        JLabel lblZ_1 = new JLabel("Z");
        panelCameraPose.add(lblZ_1, "8, 4");
        
        JLabel lblTop = new JLabel("Top");
        panelCameraPose.add(lblTop, "2, 6, right, default");
        
        textFieldCamPoseTopX = new JTextField();
        panelCameraPose.add(textFieldCamPoseTopX, "4, 6, fill, default");
        textFieldCamPoseTopX.setColumns(10);
        
        textFieldCamPoseTopY = new JTextField();
        panelCameraPose.add(textFieldCamPoseTopY, "6, 6, fill, default");
        textFieldCamPoseTopY.setColumns(10);
        
        textFieldCamPoseTopZ = new JTextField();
        panelCameraPose.add(textFieldCamPoseTopZ, "8, 6, fill, default");
        textFieldCamPoseTopZ.setColumns(10);
        
        LocationButtonsPanel locationButtonsPanelCamPoseTop = new LocationButtonsPanel(textFieldCamPoseTopX, textFieldCamPoseTopY, textFieldCamPoseTopZ, null);
        panelCameraPose.add(locationButtonsPanelCamPoseTop, "10, 6, left, default");
        
        JLabel lblBottom = new JLabel("Bottom");
        panelCameraPose.add(lblBottom, "2, 8, right, default");
        
        textFieldCamPoseBottomX = new JTextField();
        panelCameraPose.add(textFieldCamPoseBottomX, "4, 8, fill, default");
        textFieldCamPoseBottomX.setColumns(10);
        
        textFieldCamPoseBottomY = new JTextField();
        panelCameraPose.add(textFieldCamPoseBottomY, "6, 8, fill, default");
        textFieldCamPoseBottomY.setColumns(10);
        
        textFieldCamPoseBottomZ = new JTextField();
        panelCameraPose.add(textFieldCamPoseBottomZ, "8, 8, fill, default");
        textFieldCamPoseBottomZ.setColumns(10);
        
        LocationButtonsPanel locationButtonsPanelCamPoseBottom = new LocationButtonsPanel(textFieldCamPoseBottomX, textFieldCamPoseBottomY, textFieldCamPoseBottomZ, null);
        panelCameraPose.add(locationButtonsPanelCamPoseBottom, "10, 8, left, default");
        
        JPanel panelTools = new JPanel();
        panelTools.setBorder(new TitledBorder(null, "Tools", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelTools);
        panelTools.setLayout(new FormLayout(new ColumnSpec[] {
                FormFactory.RELATED_GAP_COLSPEC,
                FormFactory.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormFactory.RELATED_GAP_ROWSPEC,
                FormFactory.DEFAULT_ROWSPEC,}));
        
        chckbxAutoDetectTool = new JCheckBox("Auto Detect Tool Offsets on First Move (EXPERIMENTAL)");
        panelTools.add(chckbxAutoDetectTool, "2, 2");
        
        JPanel panelTerminal = new JPanel();
        panelTerminal.setBorder(new TitledBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null), "FireStep Terminal", TitledBorder.LEADING, TitledBorder.TOP, null, new Color(0, 0, 0)));
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
                RowSpec.decode("100dlu"),}));
        
        terminalCommandTextField = new JTextField();
        panelTerminal.add(terminalCommandTextField, "2, 2, fill, default");
        terminalCommandTextField.setColumns(10);
        terminalCommandTextField.setAction(terminalSend);
        terminalCommandTextField.addKeyListener(new KeyAdapter() {
            @Override
            public void keyReleased(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_UP) {
                    if (++historyIndex >= history.size()) {
                        historyIndex = history.size() - 1;
                    }
                    terminalCommandTextField.setText(history.get(historyIndex));
                }
                else if (e.getKeyCode() == KeyEvent.VK_DOWN) {
                    if (--historyIndex < 0) {
                        historyIndex = 0;
                    }
                    terminalCommandTextField.setText(history.get(historyIndex));
                }
                else {
                    super.keyTyped(e);
                }
            }
        });
        
        JButton btnSend = new JButton(terminalSend);
        panelTerminal.add(btnSend, "4, 2");
        
        JScrollPane scrollPane_1 = new JScrollPane();
        panelTerminal.add(scrollPane_1, "2, 4, 3, 1, fill, fill");
        
        terminalLogTextPane = new JTextPane();
        scrollPane_1.setViewportView(terminalLogTextPane);
    }
 
    @Override
    public void createBindings() {
        super.createBindings();
        LengthConverter lengthConverter = new LengthConverter();
        IntegerConverter intConverter = new IntegerConverter();
        DoubleConverter doubleConverter = new DoubleConverter(Configuration.get().getLengthDisplayFormat());
        RotatableDeltaKinematicsCalculator calc = driver.getDeltaCalculator();

        addWrappedBinding(driver, "useFireStepKinematics", checkBoxFireStepKinematics, "selected");
        
        addWrappedBinding(calc, "e", textFieldE, "text", doubleConverter);
        addWrappedBinding(calc, "f", textFieldF, "text", doubleConverter);
        addWrappedBinding(calc, "rE", textFieldRe, "text", doubleConverter);
        addWrappedBinding(calc, "rF", textFieldRf, "text", doubleConverter);
        addWrappedBinding(calc, "pulleyReductionX", textFieldGrX, "text", doubleConverter);
        addWrappedBinding(calc, "pulleyReductionY", textFieldGrY, "text", doubleConverter);
        addWrappedBinding(calc, "pulleyReductionZ", textFieldGrZ, "text", doubleConverter);
        addWrappedBinding(calc, "homeAngleStepsX", textFieldHascX, "text", intConverter);
        addWrappedBinding(calc, "homeAngleStepsY", textFieldHascY, "text", intConverter);
        addWrappedBinding(calc, "homeAngleStepsZ", textFieldHascZ, "text", intConverter);
        addWrappedBinding(calc, "stepsPerMotorRotation", textFieldSteps, "text", doubleConverter);
        addWrappedBinding(calc, "motorMicrosteps", textFieldMicrosteps, "text", doubleConverter);
        
        addWrappedBinding(driver, "autoUpdateToolOffsets", chckbxAutoDetectTool, "selected");
        addWrappedBinding(driver.getBarycentricCalibration(), "enabled", chckbxEnableBarycentricInterpolation, "selected");
        
        
        MutableLocationProxy camPoseTop = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, 
                driver.getCameraPoseCalibration(), 
                "top",
                camPoseTop, "location");
        addWrappedBinding(camPoseTop, "lengthX", textFieldCamPoseTopX, "text", 
                lengthConverter);
        addWrappedBinding(camPoseTop, "lengthY", textFieldCamPoseTopY, "text", 
                lengthConverter);
        addWrappedBinding(camPoseTop, "lengthZ", textFieldCamPoseTopZ, "text", 
                lengthConverter);
        
        MutableLocationProxy camPoseBottom = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, 
                driver.getCameraPoseCalibration(), 
                "bottom",
                camPoseBottom, "location");
        addWrappedBinding(camPoseBottom, "lengthX", textFieldCamPoseBottomX, "text", 
                lengthConverter);
        addWrappedBinding(camPoseBottom, "lengthY", textFieldCamPoseBottomY, "text", 
                lengthConverter);
        addWrappedBinding(camPoseBottom, "lengthZ", textFieldCamPoseBottomZ, "text", 
                lengthConverter);
        
        addWrappedBinding(driver.getCameraPoseCalibration(), "enabled", chckbxEnableCameraPose, "selected");
        

        ComponentDecorators.decorateWithAutoSelect(textFieldE);
        ComponentDecorators.decorateWithAutoSelect(textFieldF);
        ComponentDecorators.decorateWithAutoSelect(textFieldRe);
        ComponentDecorators.decorateWithAutoSelect(textFieldRf);
        ComponentDecorators.decorateWithAutoSelect(textFieldGrX);
        ComponentDecorators.decorateWithAutoSelect(textFieldGrY);
        ComponentDecorators.decorateWithAutoSelect(textFieldGrZ);
        ComponentDecorators.decorateWithAutoSelect(textFieldHascX);
        ComponentDecorators.decorateWithAutoSelect(textFieldHascY);
        ComponentDecorators.decorateWithAutoSelect(textFieldHascZ);
        ComponentDecorators.decorateWithAutoSelect(textFieldSteps);
        ComponentDecorators.decorateWithAutoSelect(textFieldMicrosteps);
        
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCamPoseTopX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCamPoseTopY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCamPoseTopZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCamPoseBottomX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCamPoseBottomY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCamPoseBottomZ);
    }

    @SuppressWarnings("serial")
    private Action probePoint = new AbstractAction("Single Z Probe") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            final Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0); //Assumes one head on the machine
            try {
                Location originalLocation = nozzle.getLocation();
                Location result = driver.probePoint((ReferenceHeadMountable) nozzle, nozzle.getLocation());
                nozzle.moveTo(originalLocation, 1.0);
                labelSingleProbeResults.setText(result.getZ() + "");
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    @SuppressWarnings("serial")
    private Action probeGrid = new AbstractAction("Detailed Z Probe") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            final Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0); //Assumes one head on the machine
            try {
                double[][] results = driver.probeGrid((ReferenceHeadMountable) nozzle);
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
                textPaneZprobeDetailedResults.setText(textPaneZprobeDetailedResults.getText() + s);
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
    private Action probeCorners = new AbstractAction("Corner Z Probe") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            final Nozzle nozzle = Configuration.get().getMachine().getHeads().get(0).getNozzles().get(0); //Assumes one head on the machine
            try {
                List<Location> results = driver.probeCorners((ReferenceHeadMountable) nozzle);
                // front left, front right, back right, back left
                Location frontLeft = results.get(0);
                Location frontRight = results.get(1);
                Location backRight = results.get(2);
                Location backLeft = results.get(3);
                labelFrontLeft.setText("" + frontLeft.subtract(frontLeft).getZ());
                labelFrontRight.setText("" + frontRight.subtract(frontLeft).getZ());
                labelBackRight.setText("" + backRight.subtract(frontLeft).getZ());
                labelBackLeft.setText("" + backLeft.subtract(frontLeft).getZ());
            }
            catch (Exception e1){
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    
    @SuppressWarnings("serial")
    private Action terminalSend = new AbstractAction("Send") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                List<JsonObject> responses = driver.sendJsonCommand(terminalCommandTextField.getText(), 30000);
                String s = terminalCommandTextField.getText() + " => ";
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
    private Action barycentricCaptureFull = new AbstractAction("Barycentric Capture (Full)") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            if (driver.getBarycentricCalibration().isEnabled()) {
                MessageBoxes.errorBox(getTopLevelAncestor(), "Error", "You can not perform Barycentric Capture with Barycentric Interpolation enabled. Please disable it and reset the machine before continuing.");
                return;
            }
            if (JOptionPane.showConfirmDialog(
                    getTopLevelAncestor(), 
                    "Make sure that you have the camera centered and focused\r\nover the center point of your calibration grid and then press Yes to start.", 
                    "Start Barycentric Capture (Full)?",
                    JOptionPane.YES_NO_OPTION) != JOptionPane.YES_OPTION) {
                return;
            }
            driver.barycentricCapture(false, new FutureCallback<Void>() {
                @Override
                public void onSuccess(Void arg0) {
                    SwingUtilities.invokeLater(new Runnable() {
                        public void run() {
                            JOptionPane.showMessageDialog(
                                    getTopLevelAncestor(),
                                    String.format(
                                            "Calibration finished.\r\n" +
                                            "%d of %d points were not able to be mapped.\r\n" +
                                            "You can run Barycentric Capture (Unmapped) to try the unmapped ones again.",
                                            driver.getBarycentricCalibration().getUnmappedGridPoints().size(),
                                            driver.getBarycentricCalibration().getGridPoints().size()));
                        }
                    });
                }
                
                @Override
                public void onFailure(final Throwable arg0) {
                    SwingUtilities.invokeLater(new Runnable() {
                        public void run() {
                            MessageBoxes.errorBox(
                                    getTopLevelAncestor(), 
                                    "Error", 
                                    arg0);
                        }
                    });
                }
            });
        }
    };
    
    @SuppressWarnings("serial")
    private Action barycentricCaptureUnmapped = new AbstractAction("Barycentric Capture (Unmapped)") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            if (driver.getBarycentricCalibration().isEnabled()) {
                MessageBoxes.errorBox(getTopLevelAncestor(), "Error", "You can not perform Barycentric Capture with Barycentric Interpolation enabled. Please disable it and reset the machine before continuing.");
                return;
            }
            if (driver.getBarycentricCalibration().getMappedGridPoints().isEmpty()) {
                MessageBoxes.errorBox(getTopLevelAncestor(), "Error", "You have not yet performed a Full Barycentric Capture. Please do that first.");
                return;
            }
            if (driver.getBarycentricCalibration().getUnmappedGridPoints().isEmpty()) {
                MessageBoxes.errorBox(getTopLevelAncestor(), "Error", "All points are mapped. Nothing to do.");
                return;
            }
            if (JOptionPane.showConfirmDialog(
                    getTopLevelAncestor(), 
                    "This calibration will capture only the points that failed\r\n" +
                    "during the last Full Barycentric Capture. Make sure your calibration\r\n" +
                    "grid has not moved since the last Full Barycentric Capture.\r\n\r\n" +
                    "If it has moved, you should do a Full Barycentric Capture instead.\r\n\r\n" +
                    "Ready to start?", 
                    "Start Barycentric Capture (Unmapped)?",
                    JOptionPane.YES_NO_OPTION) != JOptionPane.YES_OPTION) {
                return;
            }
            driver.barycentricCapture(true, new FutureCallback<Void>() {
                @Override
                public void onSuccess(Void arg0) {
                    SwingUtilities.invokeLater(new Runnable() {
                        public void run() {
                            JOptionPane.showMessageDialog(
                                    getTopLevelAncestor(),
                                    String.format(
                                            "Calibration finished.\r\n" +
                                            "%d of %d points were not able to be mapped.\r\n" +
                                            "You can run Barycentric Capture (Unmapped) to try the unmapped ones again.",
                                            driver.getBarycentricCalibration().getUnmappedGridPoints().size(),
                                            driver.getBarycentricCalibration().getGridPoints().size()));
                        }
                    });
                }
                
                @Override
                public void onFailure(final Throwable arg0) {
                    SwingUtilities.invokeLater(new Runnable() {
                        public void run() {
                            MessageBoxes.errorBox(
                                    getTopLevelAncestor(), 
                                    "Error", 
                                    arg0);
                        }
                    });
                }
            });
        }
    };
    
    @SuppressWarnings("serial")
    private Action hascRead = new AbstractAction("Read") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                List<JsonObject> responses = driver.sendJsonCommand("{'xpo':'','ypo':'','zpo':''}");
                JsonObject response = responses.get(0).getAsJsonObject().get("r").getAsJsonObject();
                int x = response.get("xpo").getAsInt();
                int y = response.get("ypo").getAsInt();
                int z = response.get("zpo").getAsInt();
                textFieldHascX.setText("" + x);
                textFieldHascY.setText("" + y);
                textFieldHascZ.setText("" + z);
            }
            catch (Exception e1){
                e1.printStackTrace();
                JOptionPane.showMessageDialog(null, e1.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    };
    
    private JTextField terminalCommandTextField;
    private JTextPane terminalLogTextPane;
    private JLabel labelFrontRight;
    private JLabel labelFrontLeft;
    private JLabel labelBackLeft;
    private JLabel labelBackRight;
    private JCheckBox chckbxAutoDetectTool;
    private JLabel labelSingleProbeResults;
    private JTextPane textPaneZprobeDetailedResults;
    private JTextField textFieldRe;
    private JTextField textFieldRf;
    private JTextField textFieldE;
    private JTextField textFieldF;
    private JTextField textFieldHascX;
    private JTextField textFieldHascY;
    private JTextField textFieldHascZ;
    private JTextField textFieldGrX;
    private JTextField textFieldGrY;
    private JTextField textFieldGrZ;
    private JCheckBox checkBoxFireStepKinematics;
    private JCheckBox chckbxEnableBarycentricInterpolation;
    private JTextField textFieldCamPoseTopX;
    private JTextField textFieldCamPoseTopY;
    private JTextField textFieldCamPoseTopZ;
    private JTextField textFieldCamPoseBottomX;
    private JTextField textFieldCamPoseBottomY;
    private JTextField textFieldCamPoseBottomZ;
    private JCheckBox chckbxEnableCameraPose;
    private JTextField textFieldSteps;
    private JTextField textFieldMicrosteps;
}
