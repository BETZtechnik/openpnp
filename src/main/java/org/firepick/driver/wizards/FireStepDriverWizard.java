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
import java.awt.Font;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.ComboBoxModel;
import javax.swing.DefaultComboBoxModel;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
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
import org.firepick.driver.Tmc2130;
import org.firepick.driver.Tmc2130.Freewheel;
import org.firepick.driver.Tmc2130.Mres;
import org.firepick.driver.Tmc2130.Pwmfreq;
import org.firepick.driver.Tmc2130.Tbl;
import org.firepick.kinematics.RotatableDeltaKinematicsCalculator;
import org.jdesktop.beansbinding.AutoBinding.UpdateStrategy;
import org.jdesktop.beansbinding.Converter;
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
import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.FormSpecs;
import com.jgoodies.forms.layout.RowSpec;
import java.awt.event.ActionListener;

public class FireStepDriverWizard  extends AbstractSerialPortDriverConfigurationWizard {
    private final FireStepDriver driver;
    private List<String> history = new ArrayList<String>();
    private int historyIndex = 0;
    
    public FireStepDriverWizard(FireStepDriver driver_) {
        super(driver_);
        this.driver = driver_;
        
        JPanel panelDelta = new JPanel();
        panelDelta.setBorder(new TitledBorder(null, "Delta Settings", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelDelta);
        panelDelta.setLayout(new FormLayout(new ColumnSpec[] {
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,}));
        
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
        
        JPanel panelTmc2130 = new JPanel();
        panelTmc2130.setBorder(new TitledBorder(null, "TMC2130 Stepper SPI Driver Settings", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelTmc2130);
        panelTmc2130.setLayout(new FormLayout(new ColumnSpec[] {
        		FormSpecs.LABEL_COMPONENT_GAP_COLSPEC,
        		FormSpecs.MIN_COLSPEC,
        		FormSpecs.LABEL_COMPONENT_GAP_COLSPEC,
        		FormSpecs.MIN_COLSPEC,
        		FormSpecs.LABEL_COMPONENT_GAP_COLSPEC,
        		ColumnSpec.decode("100px:grow"),
        		FormSpecs.LABEL_COMPONENT_GAP_COLSPEC,
        		FormSpecs.MIN_COLSPEC,
        		FormSpecs.LABEL_COMPONENT_GAP_COLSPEC,
        		ColumnSpec.decode("default:grow(2)"),
        		FormSpecs.LABEL_COMPONENT_GAP_COLSPEC,},
        	new RowSpec[] {
        		RowSpec.decode("5dlu"),
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,
        		FormSpecs.DEFAULT_ROWSPEC,}));
        
        JLabel lblx_5 = new JLabel("0x00");
        panelTmc2130.add(lblx_5, "2, 2");
        
        JLabel lblNewLabel_46 = new JLabel("GCONF.en_pwm_mode");
        panelTmc2130.add(lblNewLabel_46, "4, 2");
        
        chkTmcGconfEnpwmmode = new JCheckBox("StealthChop");
        chkTmcGconfEnpwmmode.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcGconfEnpwmmode, "6, 2");
        
        JButton btnTmcGconfWrite = new JButton("Write GCONF");
        btnTmcGconfWrite.addActionListener(new ActionListener() {
        	public void actionPerformed(ActionEvent e) {
        		try {
            		driver.getTmc2130().setGconf();
        		}
        		catch (Exception e1) {
        			e1.printStackTrace();
        		}
        	}
        });
        panelTmc2130.add(btnTmcGconfWrite, "8, 2, 1, 2");
        
        JLabel lblNewLabel_47 = new JLabel("stealthChop voltage PWM mode enabled (depending on velocity thresholds).");
        lblNewLabel_47.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_47, "10, 2");
        
        JLabel lblGconfsmallhysteresis = new JLabel("GCONF.small_hysteresis");
        panelTmc2130.add(lblGconfsmallhysteresis, "4, 3");
        
        cmbTmcGconfSh = new JComboBox();
        cmbTmcGconfSh.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcGconfSh.setModel(new DefaultComboBoxModel(new String[] {"0 = 1/16", "1 = 1/32"}));
        panelTmc2130.add(cmbTmcGconfSh, "6, 3, fill, default");
        
        JLabel lblNewLabel_49 = new JLabel("Hysteresis for step frequency comparison is [1/16 or 1/32]");
        lblNewLabel_49.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_49, "10, 3");
        
        JLabel lblx_3 = new JLabel("0x01");
        panelTmc2130.add(lblx_3, "2, 4");
        
        JLabel lblGstatreset = new JLabel("GSTAT.reset");
        panelTmc2130.add(lblGstatreset, "4, 4");
        
        chkTmcGstatReset = new JCheckBox("Reset");
        chkTmcGstatReset.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcGstatReset, "6, 4");
        
        JButton btnTmcGstatRead = new JButton("Read GSTAT");
        panelTmc2130.add(btnTmcGstatRead, "8, 4, 1, 3");
        
        JLabel lblNewLabel_45 = new JLabel("Indicates that the IC has been reset since the last read access to GSTAT.");
        lblNewLabel_45.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_45, "10, 4");
        
        JLabel lblGstatdrverr = new JLabel("GSTAT.drv_err");
        panelTmc2130.add(lblGstatdrverr, "4, 5");
        
        chkTmcGstatDrvErr = new JCheckBox("Driver error");
        chkTmcGstatDrvErr.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcGstatDrvErr, "6, 5");
        
        JLabel lblNewLabel_44 = new JLabel("Indicates that driver 1 has been shut down due to an error since the last read access.");
        lblNewLabel_44.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_44, "10, 5");
        
        JLabel lblGstatuvcp = new JLabel("GSTAT.uv_cp");
        panelTmc2130.add(lblGstatuvcp, "4, 6");
        
        chkTmcGstatUvcp = new JCheckBox("Undervoltage");
        chkTmcGstatUvcp.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcGstatUvcp, "6, 6");
        
        JLabel lblNewLabel_43 = new JLabel("Indicates an undervoltage on the charge pump. The driver is disabled in this case.");
        lblNewLabel_43.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_43, "10, 6");
        
        JLabel lblx_2 = new JLabel("0x04");
        panelTmc2130.add(lblx_2, "2, 7");
        
        JLabel lblIoinversion = new JLabel("IOIN.version");
        panelTmc2130.add(lblIoinversion, "4, 7");
        
        txtTmcIoinVersion = new JTextField();
        txtTmcIoinVersion.setText("0");
        panelTmc2130.add(txtTmcIoinVersion, "6, 7, fill, default");
        txtTmcIoinVersion.setColumns(10);
        
        JButton btnTmcIoinRead = new JButton("Read IOIN");
        panelTmc2130.add(btnTmcIoinRead, "8, 7");
        
        JLabel lblNewLabel_48 = new JLabel("Returns version (0x11)");
        lblNewLabel_48.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_48, "10, 7");
        
        JLabel lblx = new JLabel("0x10");
        lblx.setBackground(Color.LIGHT_GRAY);
        panelTmc2130.add(lblx, "2, 8");
        
        JLabel lblTmcIhold = new JLabel("IHOLD_RUN.IHOLD");
        panelTmc2130.add(lblTmcIhold, "4, 8");
        
        cmbTmcIhrIhold = new JComboBox();
        cmbTmcIhrIhold.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcIhrIhold.setModel(new DefaultComboBoxModel(new String[] {"0 = 1/32", "1 = 2/32", "2 = 3/32", "3 = 4/32", "4 = 5/32", "5 = 6/32", "6 = 7/32", "7 = 8/32", "8 = 9/32", "9 = 10/32", "10 = 11/32", "11 = 12/32", "12 = 13/32", "13 = 14/32", "14 = 15/32", "15 = 16/32", "16 = 17/32", "17 = 18/32", "18 = 19/32", "19 = 20/32", "20 = 21/32", "21 = 22/32", "22 = 23/32", "23 = 24/32", "24 = 25/32", "25 = 26/32", "26 = 27/32", "27 = 28/32", "28 = 29/32", "29 = 30/32", "30 = 31/32", "31 = 32/32"}));
        panelTmc2130.add(cmbTmcIhrIhold, "6, 8, fill, default");
        
        JButton btnTmcIhrWrite = new JButton("Write IHOLD_DELAY");
        btnTmcIhrWrite.addActionListener(new ActionListener() {
        	public void actionPerformed(ActionEvent e) {
        		try {
					driver.getTmc2130().setIhr();
				} catch (Exception e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
        	}
        });
        panelTmc2130.add(btnTmcIhrWrite, "8, 8, 1, 3");
        
        JLabel lblIholdBlahBlah = new JLabel("Standstill current");
        lblIholdBlahBlah.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblIholdBlahBlah, "10, 8");
        
        JLabel lblTmcIrun = new JLabel("IHOLD_RUN.IRUN");
        panelTmc2130.add(lblTmcIrun, "4, 9");
        
        cmbTmcIhrIrun = new JComboBox();
        cmbTmcIhrIrun.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcIhrIrun.setModel(new DefaultComboBoxModel(new String[] {"0 = 1/32", "1 = 2/32", "2 = 3/32", "3 = 4/32", "4 = 5/32", "5 = 6/32", "6 = 7/32", "7 = 8/32", "8 = 9/32", "9 = 10/32", "10 = 11/32", "11 = 12/32", "12 = 13/32", "13 = 14/32", "14 = 15/32", "15 = 16/32", "16 = 17/32", "17 = 18/32", "18 = 19/32", "19 = 20/32", "20 = 21/32", "21 = 22/32", "22 = 23/32", "23 = 24/32", "24 = 25/32", "25 = 26/32", "26 = 27/32", "27 = 28/32", "28 = 29/32", "29 = 30/32", "30 = 31/32", "31 = 32/32"}));
        panelTmc2130.add(cmbTmcIhrIrun, "6, 9, fill, default");
        
        JLabel lblNewLabel_1 = new JLabel("Motor run current");
        lblNewLabel_1.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_1, "10, 9");
        
        JLabel lblTmcIholdDelay = new JLabel("IHOLD_RUN.IHOLD_DELAY");
        panelTmc2130.add(lblTmcIholdDelay, "4, 10");
        
        cmbTmcIhrIholddelay = new JComboBox();
        cmbTmcIhrIholddelay.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcIhrIholddelay.setModel(new DefaultComboBoxModel(new String[] {"0 = Instant power-down", "1 = 20 milliseconds", "2 = 40 milliseconds", "3 = 61 milliseconds", "4 = 81 milliseconds", "5 = 101 milliseconds", "6 = 121 milliseconds", "7 = 141 milliseconds", "8 = 161 milliseconds", "9 = 181 milliseconds", "10 = 202 milliseconds", "11 = 222 milliseconds", "12 = 242 milliseconds", "13 = 262 milliseconds", "14 = 282 milliseconds", "15 = 302 milliseconds"}));
        panelTmc2130.add(cmbTmcIhrIholddelay, "6, 10, fill, default");
        
        JLabel lblNewLabel_2 = new JLabel("Controls the number of clock cycles for motor power down after a motion as soon as standstill is detected (stst=1) and TPOWERDOWN has expired. \nThe smooth transition avoids a motor jerk upon power down.");
        lblNewLabel_2.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_2, "10, 10");
        
        JLabel lblx_1 = new JLabel("0x11");
        panelTmc2130.add(lblx_1, "2, 11");
        
        JLabel lblTmcTpowerDown = new JLabel("TPOWERDOWN");
        panelTmc2130.add(lblTmcTpowerDown, "4, 11");
        
        txtTmcTpowerdown = new JTextField();
        txtTmcTpowerdown.setText("0");
        panelTmc2130.add(txtTmcTpowerdown, "6, 11");
        txtTmcTpowerdown.setColumns(10);
        
        JButton btnTmcTpowerdownWrite = new JButton("Write TPOWERDOWN");
        panelTmc2130.add(btnTmcTpowerdownWrite, "8, 11");
        
        JLabel lblNewLabel_3 = new JLabel("(Seconds) Sets the delay time after stand still (stst) of the motor to motor current power down.");
        lblNewLabel_3.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_3, "10, 11");
        
        JLabel lblTmcTstepReg = new JLabel("0x12");
        panelTmc2130.add(lblTmcTstepReg, "2, 12");
        
        JLabel lblTmcTstep = new JLabel("TSTEP");
        panelTmc2130.add(lblTmcTstep, "4, 12");
        
        txtTmcTstep = new JTextField();
        txtTmcTstep.setText("0");
        txtTmcTstep.setToolTipText("All TSTEP related thresholds use a hysteresis of 1/16 of the compare value to compensate for jitter in the clock or the step frequency. The flag small_hysteresis modifies the hysteresis to a smaller value of 1/32.\n\n(Txxx*15/16)-1 or (Txxx*31/32)-1 is used as a second compare value for each\ncomparison value.\n\nThis means, that the lower switching velocity equals the calculated setting, but the upper switching velocity is higher as defined by the hysteresis setting.");
        panelTmc2130.add(txtTmcTstep, "6, 12");
        txtTmcTstep.setColumns(10);
        
        JButton btnTmcTstepWrite = new JButton("Read TSTEP");
        panelTmc2130.add(btnTmcTstepWrite, "8, 12");
        
        JLabel lblNewLabel_4 = new JLabel("Actual measured time between two 1/256 microsteps derived from the step input frequency in units of 1/fCLK. Measured value is (2^20)-1 in case of overflow or stand still.");
        lblNewLabel_4.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_4, "10, 12");
        
        JLabel lblTmcTpwmThrsReg = new JLabel("0x13");
        panelTmc2130.add(lblTmcTpwmThrsReg, "2, 13");
        
        JLabel lblTmcTpwmThrs = new JLabel("TPWMTHRS");
        panelTmc2130.add(lblTmcTpwmThrs, "4, 13");
        
        txtTmcPwmThrs = new JTextField();
        txtTmcPwmThrs.setText("0");
        txtTmcPwmThrs.setToolTipText("STEP ≥ TPWMTHRS\n - stealthChop PWM mode is enabled, if configured\n - dcStep is disabled");
        panelTmc2130.add(txtTmcPwmThrs, "6, 13");
        txtTmcPwmThrs.setColumns(10);
        
        JButton btnTpwmthrsWrite = new JButton("Write TPWMTHRS");
        panelTmc2130.add(btnTpwmthrsWrite, "8, 13");
        
        JLabel lblNewLabel_5 = new JLabel("This is the upper velocity for stealthChop voltage PWM mode.");
        lblNewLabel_5.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_5, "10, 13");
        
        JLabel lblTmcTcoolThrsReg = new JLabel("0x14");
        panelTmc2130.add(lblTmcTcoolThrsReg, "2, 14");
        
        JLabel lblTmcTcoolThrs = new JLabel("TCOOLTHRS");
        panelTmc2130.add(lblTmcTcoolThrs, "4, 14");
        
        txtTmcTcoolthrs = new JTextField();
        txtTmcTcoolthrs.setText("0");
        txtTmcTcoolthrs.setToolTipText("Set this parameter to disable coolStep at low speeds, where it\ncannot work reliably. The stall detection and stallGuard output\nsignal becomes enabled when exceeding this velocity. In non-\ndcStep mode, it becomes disabled again once the velocity falls below this threshold.\n\nTCOOLTHRS ≥ TSTEP ≥ THIGH:\n- coolStep is enabled, if configured\n- stealthChop voltage PWM mode is disabled\n\nTCOOLTHRS ≥ TSTEP\n- Stop on stall and stall output signal is enabled, if\nconfigured");
        panelTmc2130.add(txtTmcTcoolthrs, "6, 14");
        txtTmcTcoolthrs.setColumns(10);
        
        JButton btnTmcTcoolthrsWrite = new JButton("Write TCOOLTHRS");
        panelTmc2130.add(btnTmcTcoolthrsWrite, "8, 14");
        
        JLabel lblNewLabel_6 = new JLabel("This is the lower threshold velocity for switching on smart energy coolStep and stallGuard feature. (unsigned)");
        lblNewLabel_6.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_6, "10, 14");
        
        JLabel lblTmcThighReg = new JLabel("0x15");
        panelTmc2130.add(lblTmcThighReg, "2, 15");
        
        JLabel lblTmcThigh = new JLabel("THIGH");
        panelTmc2130.add(lblTmcThigh, "4, 15");
        
        txtTmcThigh = new JTextField();
        txtTmcThigh.setText("0");
        txtTmcThigh.setToolTipText("The stall detection feature becomes switched off for 2-3 electrical periods whenever passing THIGH threshold to compensate for the effect of switching modes.\n\nTSTEP ≤ THIGH:\n- coolStep is disabled (motor runs with normal current scale)\n- stealthChop voltage PWM mode is disabled\n- If vhighchm is set, the chopper switches to chm=1 with TFD=0 (constant off time with slow decay, only).\n- chopSync2 is switched off (SYNC=0)\n- If vhighfs is set, the motor operates in fullstep mode and the stall detection becomes switched over to dcStep stall detection.");
        panelTmc2130.add(txtTmcThigh, "6, 15");
        txtTmcThigh.setColumns(10);
        
        JButton btnTmcThighWrite = new JButton("Write THIGH");
        panelTmc2130.add(btnTmcThighWrite, "8, 15");
        
        JLabel lblNewLabel_7 = new JLabel("Allows velocity dependent switching into a different chopper mode and fullstepping to maximize torque. (unsigned)");
        lblNewLabel_7.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_7, "10, 15");
        
        JLabel lblTmcChopConfReg = new JLabel("0x6C");
        panelTmc2130.add(lblTmcChopConfReg, "2, 16");
        
        JLabel lblTmcChopConf = new JLabel("CHOPCONF.toff");
        panelTmc2130.add(lblTmcChopConf, "4, 16");
        
        cmbTmcChopchonfToff = new JComboBox();
        cmbTmcChopchonfToff.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfToff.setModel(new DefaultComboBoxModel(new String[] {"0 = Driver disable, all bridges off", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"}));
        panelTmc2130.add(cmbTmcChopchonfToff, "6, 16, fill, default");
        
        JButton btnTmcChopconfWrite = new JButton("Write CHOPCONF");
        btnTmcChopconfWrite.addActionListener(new ActionListener() {
        	public void actionPerformed(ActionEvent e) {
        		try {
					driver.getTmc2130().setChopConf();
				} catch (Exception e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
        	}
        });
        panelTmc2130.add(btnTmcChopconfWrite, "8, 16, 1, 16");
        
        JLabel lblNewLabel_8 = new JLabel("TOFF off time and driver enable");
        lblNewLabel_8.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_8, "10, 16");
        
        JLabel lblChopconf = new JLabel("CHOPCONF.hstrt");
        panelTmc2130.add(lblChopconf, "4, 17");
        
        cmbTmcChopchonfHstrt = new JComboBox();
        cmbTmcChopchonfHstrt.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfHstrt.setModel(new DefaultComboBoxModel(new String[] {"1", "2", "3", "4", "5", "6", "7", "8"}));
        panelTmc2130.add(cmbTmcChopchonfHstrt, "6, 17, fill, default");
        
        JLabel lblNewLabel_18 = new JLabel("HSTRT hysteresis start value added to HEND");
        lblNewLabel_18.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_18, "10, 17");
        
        JLabel lblNewLabel_14 = new JLabel("CHOPCONF.hend");
        panelTmc2130.add(lblNewLabel_14, "4, 18");
        
        cmbTmcChopchonfHend = new JComboBox();
        cmbTmcChopchonfHend.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfHend.setModel(new DefaultComboBoxModel(new String[] {"-3", "-2", "-1", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"}));
        panelTmc2130.add(cmbTmcChopchonfHend, "6, 18, fill, default");
        
        JLabel lblNewLabel_19 = new JLabel("HEND hysteresis low value OFFSET sine wave offset");
        lblNewLabel_19.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_19, "10, 18");
        
        JLabel lblNewLabel_15 = new JLabel("CHOPCONF.fd3");
        panelTmc2130.add(lblNewLabel_15, "4, 19");
        
        cmbTmcChopchonfFd3 = new JComboBox();
        cmbTmcChopchonfFd3.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfFd3.setModel(new DefaultComboBoxModel(new String[] {"0 = ?", "1 = MSB of fast decay time setting TFD (when chm=1)"}));
        panelTmc2130.add(cmbTmcChopchonfFd3, "6, 19, fill, default");
        
        JLabel lblNewLabel_20 = new JLabel("MSB of fast decay time setting TFD");
        lblNewLabel_20.setToolTipText("In chm=1: MSB of fast decay time setting TFD");
        lblNewLabel_20.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_20, "10, 19");
        
        JLabel lblNewLabel_16 = new JLabel("CHOPCONF.disfdcc");
        panelTmc2130.add(lblNewLabel_16, "4, 20");
        
        cmbTmcChopchonfDisfdcc = new JComboBox();
        cmbTmcChopchonfDisfdcc.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfDisfdcc.setModel(new DefaultComboBoxModel(new String[] {"0 = ", "1 = Disables current comparator usage for termination of the fast decay cycle"}));
        panelTmc2130.add(cmbTmcChopchonfDisfdcc, "6, 20, fill, default");
        
        JLabel lblNewLabel_21 = new JLabel("fast decay mode");
        lblNewLabel_21.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_21, "10, 20");
        
        JLabel lblChopconf_1 = new JLabel("CHOPCONF.rndtf");
        panelTmc2130.add(lblChopconf_1, "4, 21");
        
        cmbTmcChopchonfRndtf = new JComboBox();
        cmbTmcChopchonfRndtf.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfRndtf.setModel(new DefaultComboBoxModel(new String[] {"0 = Chopper off time is fixed as set by TOFF", "1 = Random mode, TOFF is random modulated by dNclk = -12 ... +3 clocks."}));
        panelTmc2130.add(cmbTmcChopchonfRndtf, "6, 21, fill, default");
        
        JLabel lblNewLabel_22 = new JLabel("random TOFF time");
        lblNewLabel_22.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_22, "10, 21");
        
        JLabel lblChopconf_2 = new JLabel("CHOPCONF.chm");
        panelTmc2130.add(lblChopconf_2, "4, 22");
        
        cmbTmcChopchonfChm = new JComboBox();
        cmbTmcChopchonfChm.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfChm.setModel(new DefaultComboBoxModel(new String[] {"0 = spreadCycle (standard mode)", "1 = Constant off time with fast decay time."}));
        panelTmc2130.add(cmbTmcChopchonfChm, "6, 22, fill, default");
        
        JLabel lblNewLabel_23 = new JLabel("chopper mode (SpreadCycle or ctOff mode)");
        lblNewLabel_23.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_23, "10, 22");
        
        JLabel lblChopconf_3 = new JLabel("CHOPCONF.tbl");
        panelTmc2130.add(lblChopconf_3, "4, 23");
        
        cmbTmcChopchonfTbl = new JComboBox();
        cmbTmcChopchonfTbl.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfTbl.setToolTipText("Hint: 24 or 36 is recommended for most applications");
        cmbTmcChopchonfTbl.setModel(new DefaultComboBoxModel(Tbl.values()));
        panelTmc2130.add(cmbTmcChopchonfTbl, "6, 23, fill, default");
        
        JLabel lblNewLabel_24 = new JLabel("TBL blank time select");
        lblNewLabel_24.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_24, "10, 23");
        
        JLabel lblChopconf_4 = new JLabel("CHOPCONF.vsense");
        panelTmc2130.add(lblChopconf_4, "4, 24");
        
        cmbTmcChopchonfVsense = new JComboBox();
        cmbTmcChopchonfVsense.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfVsense.setModel(new DefaultComboBoxModel(new String[] {"0 = Low sensitivity, high sense resistor voltage", "1 = High sensitivity, low sense resistor voltage"}));
        panelTmc2130.add(cmbTmcChopchonfVsense, "6, 24, fill, default");
        
        JLabel lblNewLabel_25 = new JLabel("Sense resistor voltage based current scaling");
        lblNewLabel_25.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_25, "10, 24");
        
        JLabel lblChopconf_5 = new JLabel("CHOPCONF.vhighfs");
        panelTmc2130.add(lblChopconf_5, "4, 25");
        
        cmbTmcChopchonfVhighfs = new JComboBox();
        cmbTmcChopchonfVhighfs.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfVhighfs.setModel(new DefaultComboBoxModel(new String[] {"0", "1"}));
        panelTmc2130.add(cmbTmcChopchonfVhighfs, "6, 25, fill, default");
        
        JLabel lblNewLabel_26 = new JLabel("High velocity fullstep selection");
        lblNewLabel_26.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_26, "10, 25");
        
        JLabel lblChopconf_6 = new JLabel("CHOPCONF.vhighchm");
        panelTmc2130.add(lblChopconf_6, "4, 26");
        
        cmbTmcChopchonfVhighchm = new JComboBox();
        cmbTmcChopchonfVhighchm.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfVhighchm.setModel(new DefaultComboBoxModel(new String[] {"0 = ", "1 = Switch to chm=1 and fd=0 when VHIGH is exceeded"}));
        cmbTmcChopchonfVhighchm.setToolTipText("This bit enables switching to chm=1 and fd=0, when VHIGH is exceeded. This way, a higher velocity can be achieved. Can be combined with vhighfs=1. If set, the TOFF setting automatically becomes doubled during high velocity operation in order to avoid doubling of the chopper frequency.");
        panelTmc2130.add(cmbTmcChopchonfVhighchm, "6, 26, fill, default");
        
        JLabel lblNewLabel_27 = new JLabel("high velocity chopper mode");
        lblNewLabel_27.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_27, "10, 26");
        
        JLabel lblChopconf_7 = new JLabel("CHOPCONF.sync");
        panelTmc2130.add(lblChopconf_7, "4, 27");
        
        cmbTmcChopchonfSync = new JComboBox();
        cmbTmcChopchonfSync.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfSync.setToolTipText("This register allows synchronization of the chopper for both phases of a two phase motor in  order to avoid the occurrence of a beat, especially at low motor velocities. It is automatically switched off above VHIGH.  \n\nSynchronization with f SYNC = f CLK /(sync*64) \n\nHint: Set TOFF to a low value, so that the chopper cycle is ended, before the next sync clock pulse occurs. Set for the double desired chopper frequency for chm=0, for the desired base chopper frequency for chm=1.");
        cmbTmcChopchonfSync.setModel(new DefaultComboBoxModel(new String[] {"0 = Chopper sync function chopSync off", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"}));
        panelTmc2130.add(cmbTmcChopchonfSync, "6, 27, fill, default");
        
        JLabel lblNewLabel_28 = new JLabel("SYNC PWM synchronization clock");
        lblNewLabel_28.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_28, "10, 27");
        
        JLabel lblChopconf_8 = new JLabel("CHOPCONF.mres");
        panelTmc2130.add(lblChopconf_8, "4, 28");
        
        cmbTmcChopchonfMres = new JComboBox();
        cmbTmcChopchonfMres.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfMres.setModel(new DefaultComboBoxModel(Mres.values()));
        panelTmc2130.add(cmbTmcChopchonfMres, "6, 28, fill, default");
        
        JLabel lblMicrosteppingResolution = new JLabel("MRES micro step resolution");
        lblMicrosteppingResolution.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblMicrosteppingResolution, "10, 28");
        
        JLabel lblNewLabel_17 = new JLabel("CHOPCONF.intpol");
        panelTmc2130.add(lblNewLabel_17, "4, 29");
        
        cmbTmcChopchonfIntpol = new JComboBox();
        cmbTmcChopchonfIntpol.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfIntpol.setModel(new DefaultComboBoxModel(new String[] {"0 = No Interpolation", "1 = Interpolate to 256x"}));
        panelTmc2130.add(cmbTmcChopchonfIntpol, "6, 29, fill, default");
        
        JLabel lblInterpolation = new JLabel("interpolation to 256\nmicrosteps");
        lblInterpolation.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblInterpolation, "10, 29");
        
        JLabel lblChopconf_9 = new JLabel("CHOPCONF.dedge");
        panelTmc2130.add(lblChopconf_9, "4, 30");
        
        cmbTmcChopchonfDedge = new JComboBox();
        cmbTmcChopchonfDedge.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopchonfDedge.setModel(new DefaultComboBoxModel(new String[] {"0 = ", "1 = Enable step impulse at each step edge to reduce step frequency requirement."}));
        panelTmc2130.add(cmbTmcChopchonfDedge, "6, 30, fill, default");
        
        JLabel lblEnableStepImpulse = new JLabel("Enable step impulse at each step edge to reduce step frequency requirement.");
        lblEnableStepImpulse.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblEnableStepImpulse, "10, 30");
        
        JLabel cmbTmcChopchonfDiss2g = new JLabel("CHOPCONF.diss2g");
        panelTmc2130.add(cmbTmcChopchonfDiss2g, "4, 31");
        
        cmbTmcChopconfDiss2g = new JComboBox();
        cmbTmcChopconfDiss2g.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcChopconfDiss2g.setModel(new DefaultComboBoxModel(new String[] {"0 = Short to GND protection is on", "1 = Short to GND protection is disabled"}));
        panelTmc2130.add(cmbTmcChopconfDiss2g, "6, 31, fill, default");
        
        JLabel lblShorttogroundProtection = new JLabel("Short-to-ground protection disabled");
        lblShorttogroundProtection.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblShorttogroundProtection, "10, 31");
        
        JLabel lblxd = new JLabel("0x6D");
        panelTmc2130.add(lblxd, "2, 32");
        
        JLabel lblCoolconf = new JLabel("COOLCONF.sfilt");
        panelTmc2130.add(lblCoolconf, "4, 32");
        
        cmbTmcCoolconfSfilt = new JComboBox();
        cmbTmcCoolconfSfilt.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcCoolconfSfilt.setModel(new DefaultComboBoxModel(new String[] {"0 = Standard mode, high time resolution for stallGuard2", "1 = Filtered mode, stallGuard2 signal updated for each four fullsteps (resp. six fullsteps for 3 phase motor) only to compensate for motor pole tolerances"}));
        panelTmc2130.add(cmbTmcCoolconfSfilt, "6, 32, fill, default");
        
        JButton btnTmcCoolconfWrite = new JButton("Write COOLCONF");
        btnTmcCoolconfWrite.addActionListener(new ActionListener() {
        	public void actionPerformed(ActionEvent e) {
        		try {
					driver.getTmc2130().setCoolConf();
				} catch (Exception e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
        	}
        });
        panelTmc2130.add(btnTmcCoolconfWrite, "8, 32, 1, 7");
        
        JLabel lblS = new JLabel("stallGuard2 filter enable");
        lblS.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblS, "10, 32");
        
        JLabel lblCoolconf_1 = new JLabel("COOLCONF.sgt");
        panelTmc2130.add(lblCoolconf_1, "4, 33");
        
        txtTmcCoolconfSgt = new JTextField();
        txtTmcCoolconfSgt.setText("0");
        panelTmc2130.add(txtTmcCoolconfSgt, "6, 33, fill, default");
        txtTmcCoolconfSgt.setColumns(10);
        
        JLabel lblS_1 = new JLabel("stallGuard2 threshold value");
        lblS_1.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblS_1, "10, 33");
        
        JLabel lblCoolconfseimin = new JLabel("COOLCONF.seimin");
        panelTmc2130.add(lblCoolconfseimin, "4, 34");
        
        cmbTmcCoolconfSeimin = new JComboBox();
        cmbTmcCoolconfSeimin.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcCoolconfSeimin.setModel(new DefaultComboBoxModel(new String[] {"0 = 1/2 of current setting (IRUN)", "1 = 1/4 of current setting (IRUN)"}));
        panelTmc2130.add(cmbTmcCoolconfSeimin, "6, 34, fill, default");
        
        JLabel lblNewLabel_51 = new JLabel("Minimum current for smart current control");
        lblNewLabel_51.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_51, "10, 34");
        
        JLabel lblNewLabel_50 = new JLabel("COOLCONF.sedn");
        panelTmc2130.add(lblNewLabel_50, "4, 35");
        
        cmbTmcCoolconfSedn = new JComboBox();
        cmbTmcCoolconfSedn.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcCoolconfSedn.setModel(new DefaultComboBoxModel(new String[] {"0 = For each 32 stallGuard2 values decrease by one", "1 = For each 8 stallGuard2 values decrease by one", "2 = For each 2 stallGuard2 values decrease by one", "3 = For each stallGuard2 value decrease by one"}));
        panelTmc2130.add(cmbTmcCoolconfSedn, "6, 35, fill, default");
        
        JLabel lblNewLabel_52 = new JLabel("Current down step speed");
        lblNewLabel_52.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_52, "10, 35");
        
        JLabel lblCoolconfsemax = new JLabel("COOLCONF.semax");
        panelTmc2130.add(lblCoolconfsemax, "4, 36");
        
        cmbTmcCoolconfSemax = new JComboBox();
        cmbTmcCoolconfSemax.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcCoolconfSemax.setModel(new DefaultComboBoxModel(new String[] {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"}));
        panelTmc2130.add(cmbTmcCoolconfSemax, "6, 36, fill, default");
        
        JLabel lblNewLabel_53 = new JLabel("stallGuard2 hysteresis value for smart current control");
        lblNewLabel_53.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_53, "10, 36");
        
        JLabel lblCoolconfseup = new JLabel("COOLCONF.seup");
        panelTmc2130.add(lblCoolconfseup, "4, 37");
        
        cmbTmcCoolconfSeup = new JComboBox();
        cmbTmcCoolconfSeup.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcCoolconfSeup.setModel(new DefaultComboBoxModel(new String[] {"0 = 1", "1 = 2", "2 = 4", "3 = 8"}));
        panelTmc2130.add(cmbTmcCoolconfSeup, "6, 37, fill, default");
        
        JLabel lblNewLabel_54 = new JLabel("Current up step width");
        lblNewLabel_54.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_54, "10, 37");
        
        JLabel lblCoolconfsemin = new JLabel("COOLCONF.semin");
        panelTmc2130.add(lblCoolconfsemin, "4, 38");
        
        cmbTmcCoolconfSemin = new JComboBox();
        cmbTmcCoolconfSemin.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcCoolconfSemin.setModel(new DefaultComboBoxModel(new String[] {"0 = Smart current control coolStep off", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"}));
        panelTmc2130.add(cmbTmcCoolconfSemin, "6, 38, fill, default");
        
        JLabel lblNewLabel_55 = new JLabel("Minimum stallGuard2 value for smart current control and smart current enable");
        lblNewLabel_55.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_55, "10, 38");
        
        JLabel lblxf = new JLabel("0x6F");
        panelTmc2130.add(lblxf, "2, 39");
        
        JLabel lblTmcDrvStatus = new JLabel("DRV_STATUS.stst");
        panelTmc2130.add(lblTmcDrvStatus, "4, 39");
        
        chkTmcDrvstatusStst = new JCheckBox("Standstill");
        chkTmcDrvstatusStst.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcDrvstatusStst, "6, 39");
        
        JButton btnTmcDrvstatusRead = new JButton("Read DRV_STATUS");
        panelTmc2130.add(btnTmcDrvstatusRead, "8, 39, 1, 11");
        
        JLabel lblNewLabel_9 = new JLabel("standstill indicator");
        lblNewLabel_9.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_9, "10, 39");
        
        JLabel lblDrvstatus = new JLabel("DRV_STATUS.ola");
        panelTmc2130.add(lblDrvstatus, "4, 40");
        
        chkTmcDrvstatusOla = new JCheckBox("Open Load");
        chkTmcDrvstatusOla.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcDrvstatusOla, "6, 40");
        
        JLabel lblNewLabel_42 = new JLabel("open load indicator phase B");
        lblNewLabel_42.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_42, "10, 40");
        
        JLabel lblDrvstatus_2 = new JLabel("DRV_STATUS.olb");
        panelTmc2130.add(lblDrvstatus_2, "4, 41");
        
        chkTmcDrvstatusOlb = new JCheckBox("Open Load");
        chkTmcDrvstatusOlb.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcDrvstatusOlb, "6, 41");
        
        JLabel lblNewLabel_41 = new JLabel("open load indicator phase A");
        lblNewLabel_41.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_41, "10, 41");
        
        JLabel lblDrvstatus_3 = new JLabel("DRV_STATUS.s2gb");
        panelTmc2130.add(lblDrvstatus_3, "4, 42");
        
        chkTmcDrvstatusS2gb = new JCheckBox("Short 2 GND");
        chkTmcDrvstatusS2gb.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcDrvstatusS2gb, "6, 42");
        
        JLabel lblNewLabel_40 = new JLabel("short to ground indicator phase B");
        lblNewLabel_40.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_40, "10, 42");
        
        JLabel lblDrvstatus_7 = new JLabel("DRV_STATUS.s2ga");
        panelTmc2130.add(lblDrvstatus_7, "4, 43");
        
        chkTmcDrvstatusS2ga = new JCheckBox("Short 2 GND");
        chkTmcDrvstatusS2ga.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcDrvstatusS2ga, "6, 43");
        
        JLabel lblNewLabel_39 = new JLabel("short to ground indicator phase A");
        lblNewLabel_39.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_39, "10, 43");
        
        JLabel lblDrvstatus_4 = new JLabel("DRV_STATUS.otpw");
        panelTmc2130.add(lblDrvstatus_4, "4, 44");
        
        chkTmcDrvstatusOtpw = new JCheckBox("OT pre-warn");
        chkTmcDrvstatusOtpw.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcDrvstatusOtpw, "6, 44");
        
        JLabel lblNewLabel_38 = new JLabel("overtemperature pre-warning flag");
        lblNewLabel_38.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_38, "10, 44");
        
        JLabel lblDrvstatus_5 = new JLabel("DRV_STATUS.ot");
        panelTmc2130.add(lblDrvstatus_5, "4, 45");
        
        chkTmcDrvstatusOt = new JCheckBox("Overtemp");
        chkTmcDrvstatusOt.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcDrvstatusOt, "6, 45");
        
        JLabel lblNewLabel_37 = new JLabel("overtemperature flag");
        lblNewLabel_37.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_37, "10, 45");
        
        JLabel lblDrvstatus_6 = new JLabel("DRV_STATUS.stallGuard");
        panelTmc2130.add(lblDrvstatus_6, "4, 46");
        
        chkTmcDrvstatusStallguard = new JCheckBox("Stall Detected");
        chkTmcDrvstatusStallguard.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcDrvstatusStallguard, "6, 46");
        
        JLabel lblNewLabel_36 = new JLabel("stallGuard2 status");
        lblNewLabel_36.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_36, "10, 46");
        
        JLabel lblDrvstatus_8 = new JLabel("DRV_STATUS.cs_actual");
        panelTmc2130.add(lblDrvstatus_8, "4, 47");
        
        txtTmcDrvstatusCsactual = new JTextField();
        txtTmcDrvstatusCsactual.setText("0");
        panelTmc2130.add(txtTmcDrvstatusCsactual, "6, 47, fill, default");
        txtTmcDrvstatusCsactual.setColumns(10);
        
        JLabel lblNewLabel_35 = new JLabel("actual motor current / smart energy current");
        lblNewLabel_35.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_35, "10, 47");
        
        JLabel lblDrvstatus_9 = new JLabel("DRV_STATUS.fsactive");
        panelTmc2130.add(lblDrvstatus_9, "4, 48");
        
        chkTmcDrvstatusFsactive = new JCheckBox("Active");
        chkTmcDrvstatusFsactive.setFont(new Font("Dialog", Font.PLAIN, 10));
        panelTmc2130.add(chkTmcDrvstatusFsactive, "6, 48");
        
        JLabel lblNewLabel_34 = new JLabel("full step active indicator");
        lblNewLabel_34.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_34, "10, 48");
        
        JLabel lblDrvstatus_1 = new JLabel("DRV_STATUS.sg_result");
        panelTmc2130.add(lblDrvstatus_1, "4, 49");
        
        txtTmcDrvstatusSgresult = new JTextField();
        txtTmcDrvstatusSgresult.setText("0");
        panelTmc2130.add(txtTmcDrvstatusSgresult, "6, 49, fill, default");
        txtTmcDrvstatusSgresult.setColumns(10);
        
        JLabel lblNewLabel_33 = new JLabel("Mechanical load measurement");
        lblNewLabel_33.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_33, "10, 49");
        
        JLabel lblNewLabel = new JLabel("0x70");
        panelTmc2130.add(lblNewLabel, "2, 50");
        
        JLabel lblPwmconf = new JLabel("PWMCONF.pwm_ampl");
        panelTmc2130.add(lblPwmconf, "4, 50");
        
        txtTmcPwmconfAmpl = new JTextField();
        panelTmc2130.add(txtTmcPwmconfAmpl, "6, 50, fill, default");
        txtTmcPwmconfAmpl.setColumns(10);
        
        JButton btnTmcPwmconfWrite = new JButton("Write PWMCONF");
        panelTmc2130.add(btnTmcPwmconfWrite, "8, 50, 1, 6");
        
        JLabel lblNewLabel_10 = new JLabel("User defined amplitude (offset)");
        lblNewLabel_10.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_10, "10, 50");
        
        JLabel lblNewLabel_12 = new JLabel("PWMCONF.pwm_grad");
        panelTmc2130.add(lblNewLabel_12, "4, 51");
        
        txtTmcPwmconfGrad = new JTextField();
        panelTmc2130.add(txtTmcPwmconfGrad, "6, 51, fill, default");
        txtTmcPwmconfGrad.setColumns(10);
        
        JLabel lblNewLabel_13 = new JLabel("User defined amplitude (gradient) or regulation loop gradient");
        lblNewLabel_13.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_13, "10, 51");
        
        JLabel lblPwmconfpwmfreq = new JLabel("PWMCONF.pwm_freq");
        panelTmc2130.add(lblPwmconfpwmfreq, "4, 52");
        
        cmbTmcPwmconfFreq = new JComboBox();
        cmbTmcPwmconfFreq.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcPwmconfFreq.setModel(new DefaultComboBoxModel(Pwmfreq.values()));
        panelTmc2130.add(cmbTmcPwmconfFreq, "6, 52, fill, default");
        
        JLabel lblNewLabel_29 = new JLabel("PWM frequency selection");
        lblNewLabel_29.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_29, "10, 52");
        
        JLabel lblPwmconfpwmautoscale = new JLabel("PWMCONF.pwm_autoscale");
        panelTmc2130.add(lblPwmconfpwmautoscale, "4, 53");
        
        chkTmcPwmconfAutoscale = new JCheckBox("Autoscale?");
        chkTmcPwmconfAutoscale.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(chkTmcPwmconfAutoscale, "6, 53");
        
        JLabel lblNewLabel_11 = new JLabel("PWM automatic amplitude scaling");
        lblNewLabel_11.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_11, "10, 53");
        
        JLabel lblPwmconfpwmsymmetric = new JLabel("PWMCONF.pwm_symmetric");
        panelTmc2130.add(lblPwmconfpwmsymmetric, "4, 54, right, default");
        
        cmbTmcPwmconfSymmetric = new JComboBox();
        cmbTmcPwmconfSymmetric.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcPwmconfSymmetric.setModel(new DefaultComboBoxModel(new String[] {"Standard", "Symmetric"}));
        panelTmc2130.add(cmbTmcPwmconfSymmetric, "6, 54, fill, default");
        
        JLabel lblNewLabel_30 = new JLabel("Force symmetric PWM");
        lblNewLabel_30.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_30, "10, 54");
        
        JLabel lblPwmconffreewheel = new JLabel("PWMCONF.freewheel");
        panelTmc2130.add(lblPwmconffreewheel, "4, 55");
        
        cmbTmcPwmconfFreewheel = new JComboBox();
        cmbTmcPwmconfFreewheel.setFont(new Font("Dialog", Font.PLAIN, 10));
        cmbTmcPwmconfFreewheel.setModel(new DefaultComboBoxModel(Freewheel.values()));
        panelTmc2130.add(cmbTmcPwmconfFreewheel, "6, 55, fill, default");
        
        JLabel lblNewLabel_31 = new JLabel("Allows different standstill modes");
        lblNewLabel_31.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_31, "10, 55");
        
        JLabel lblx_4 = new JLabel("0x71");
        panelTmc2130.add(lblx_4, "2, 56");
        
        JLabel lblPwmscale = new JLabel("PWM_SCALE");
        panelTmc2130.add(lblPwmscale, "4, 56");
        
        txtTmcPwmscale = new JTextField();
        panelTmc2130.add(txtTmcPwmscale, "6, 56, fill, default");
        txtTmcPwmscale.setColumns(10);
        
        JButton btnTmcPwmscaleRead = new JButton("Read PWM_SCALE");
        panelTmc2130.add(btnTmcPwmscaleRead, "8, 56");
        
        JLabel lblNewLabel_32 = new JLabel("Actual PWM scaler (255 = max voltage)");
        lblNewLabel_32.setFont(new Font("Dialog", Font.PLAIN, 12));
        panelTmc2130.add(lblNewLabel_32, "10, 56");
        
        JPanel panelBedLeveling = new JPanel();
        panelBedLeveling.setBorder(new TitledBorder(null, "Bed Leveling", TitledBorder.LEADING, TitledBorder.TOP, null, null));
        contentPanel.add(panelBedLeveling);
        panelBedLeveling.setLayout(new FormLayout(new ColumnSpec[] {
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("default:grow"),},
            new RowSpec[] {
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                RowSpec.decode("default:grow"),
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,}));
        
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
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,}));
        
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
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("default:grow"),},
            new RowSpec[] {
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                RowSpec.decode("default:grow"),
                FormSpecs.RELATED_GAP_ROWSPEC,
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
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,}));
        
        chckbxAutoDetectTool = new JCheckBox("Auto Detect Tool Offsets on First Move (EXPERIMENTAL)");
        panelTools.add(chckbxAutoDetectTool, "2, 2");
        
        JPanel panelTerminal = new JPanel();
        panelTerminal.setBorder(new TitledBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null), "FireStep Terminal", TitledBorder.LEADING, TitledBorder.TOP, null, new Color(0, 0, 0)));
        contentPanel.add(panelTerminal);
        panelTerminal.setLayout(new FormLayout(new ColumnSpec[] {
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("default:grow"),
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,},
            new RowSpec[] {
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
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
        
//        // gconf
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().gConf.en_pwm_mode, "value", chkTmcGconfEnpwmmode, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().gConf.small_hysteresis, "value", cmbTmcGconfSh, "selectedItem", new BooleanListConverter(cmbTmcGconfSh.getModel()));
//        // gstat
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().gStat.drv_err, "value", chkTmcGstatDrvErr, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().gStat.reset, "value", chkTmcGstatReset, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().gStat.uv_cp, "value", chkTmcGstatUvcp, "selected");
//        // ioin
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().ioin, "value", txtTmcIoinVersion, "text");
//        // ihr        
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().ihr.ihold, "value", cmbTmcIhrIhold, "selectedItem", new ShortPrefixConverter(cmbTmcIhrIhold.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().ihr.iholddelay, "value", cmbTmcIhrIholddelay, "selectedItem", new ShortPrefixConverter(cmbTmcIhrIholddelay.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().ihr.irun, "value", cmbTmcIhrIrun, "selectedItem", new ShortPrefixConverter(cmbTmcIhrIrun.getModel()));
//        // various
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().tPowerDown, "value", txtTmcTpowerdown, "text");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().tStep, "value", txtTmcTstep, "text");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().tPwmThrs, "value", txtTmcPwmThrs, "text");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().tCoolThrs, "value", txtTmcTcoolthrs, "text");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().tHigh, "value", txtTmcThigh, "text");
        // chopconf
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.chm, "value", cmbTmcChopchonfChm, "selectedItem", new BooleanListConverter(cmbTmcChopchonfChm.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.dedge, "value", cmbTmcChopchonfDedge, "selectedItem", new BooleanListConverter(cmbTmcChopchonfDedge.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.disfdcc, "value", cmbTmcChopchonfDisfdcc, "selectedItem", new BooleanListConverter(cmbTmcChopchonfDisfdcc.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.diss2g, "value", cmbTmcChopconfDiss2g, "selectedItem", new BooleanListConverter(cmbTmcChopconfDiss2g.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.fd3, "value", cmbTmcChopchonfFd3, "selectedItem", new BooleanListConverter(cmbTmcChopchonfFd3.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.hend, "value", cmbTmcChopchonfHend, "selectedItem", new ShortPrefixConverter(cmbTmcChopchonfHend.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.hstrt, "value", cmbTmcChopchonfHstrt, "selectedItem", new ShortPrefixConverter(cmbTmcChopchonfHstrt.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.intpol, "value", cmbTmcChopchonfIntpol, "selectedItem", new BooleanListConverter(cmbTmcChopchonfIntpol.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.mres, "value", cmbTmcChopchonfMres, "selectedItem");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.rndtf, "value", cmbTmcChopchonfRndtf, "selectedItem", new BooleanListConverter(cmbTmcChopchonfRndtf.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.sync, "value", cmbTmcChopchonfSync, "selectedItem", new ShortPrefixConverter(cmbTmcChopchonfSync.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.tbl, "value", cmbTmcChopchonfTbl, "selectedItem");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.toff, "value", cmbTmcChopchonfToff, "selectedItem", new ShortPrefixConverter(cmbTmcChopchonfToff.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.vhighchm, "value", cmbTmcChopchonfVhighchm, "selectedItem", new BooleanListConverter(cmbTmcChopchonfVhighchm.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.vhighfs, "value", cmbTmcChopchonfVhighfs, "selectedItem", new BooleanListConverter(cmbTmcChopchonfVhighfs.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().chopConf.vsense, "value", cmbTmcChopchonfVsense, "selectedItem", new BooleanListConverter(cmbTmcChopchonfVsense.getModel()));
        // coolconf
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().coolConfig.sfilt, "value", cmbTmcCoolconfSfilt, "selectedItem", new BooleanListConverter(cmbTmcCoolconfSfilt.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().coolConfig.sgt, "value", txtTmcCoolconfSgt, "text");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().coolConfig.seimin, "value", cmbTmcCoolconfSeimin, "selectedItem", new BooleanListConverter(cmbTmcCoolconfSeimin.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().coolConfig.sedn, "value", cmbTmcCoolconfSedn, "selectedItem", new ShortPrefixConverter(cmbTmcCoolconfSedn.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().coolConfig.semax, "value", cmbTmcCoolconfSemax, "selectedItem", new ShortPrefixConverter(cmbTmcCoolconfSemax.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().coolConfig.seup, "value", cmbTmcCoolconfSeup, "selectedItem", new ShortPrefixConverter(cmbTmcCoolconfSeup.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().coolConfig.semin, "value", cmbTmcCoolconfSemin, "selectedItem", new ShortPrefixConverter(cmbTmcCoolconfSemin.getModel()));
        // drvstatus
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.stst, "value", chkTmcDrvstatusStst, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.olb, "value", chkTmcDrvstatusOlb, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.ola, "value", chkTmcDrvstatusOla, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.s2gb, "value", chkTmcDrvstatusS2gb, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.s2ga, "value", chkTmcDrvstatusS2ga, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.otpw, "value", chkTmcDrvstatusOtpw, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.ot, "value", chkTmcDrvstatusOt, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.stallGuard, "value", chkTmcDrvstatusStallguard, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.cs_actual, "value", txtTmcDrvstatusCsactual, "text");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.fsactive, "value", chkTmcDrvstatusFsactive, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().drvStatus.sg_result, "value", txtTmcDrvstatusSgresult, "text");
        // pwmconf
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().pwmConf.pwm_ampl, "value", txtTmcPwmconfAmpl, "text");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().pwmConf.pwm_grad, "value", txtTmcPwmconfGrad, "text");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().pwmConf.pwm_freq, "value", cmbTmcPwmconfFreq, "selectedItem");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().pwmConf.pwm_autoscale, "value", chkTmcPwmconfAutoscale, "selected");
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().pwmConf.pwm_symmetric, "value", cmbTmcPwmconfSymmetric, "selectedItem", new BooleanListConverter(cmbTmcPwmconfSymmetric.getModel()));
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().pwmConf.freewheel, "value", cmbTmcPwmconfFreewheel, "selectedItem");
        // pwmscale
        bind(UpdateStrategy.READ_WRITE, driver.getTmc2130().pwmScale, "value", txtTmcPwmscale, "text");
        
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
    
    public class ShortPrefixConverter extends Converter<Short, String> {
    	ComboBoxModel<String> model;
    	
    	public ShortPrefixConverter(ComboBoxModel<String> model) {
    		this.model = model;
    	}
    	
		@Override
		public String convertForward(Short arg0) {
			for (int i = 0; i < model.getSize(); i++) {
				if (arg0 == getShortPrefix(model.getElementAt(i))) {
					return model.getElementAt(i);
				}
			}
			return null;
		}

		@Override
		public Short convertReverse(String arg0) {
			return getShortPrefix(arg0);
		}
		
		private short getShortPrefix(String s) {
			return Short.parseShort(s.split(" ")[0]);
		}
    }

    public class BooleanListConverter extends Converter<Boolean, String> {
    	ComboBoxModel<String> model;
    	
    	public BooleanListConverter(ComboBoxModel<String> model) {
    		this.model = model;
    	}
    	
		@Override
		public String convertForward(Boolean arg0) {
			return model.getElementAt(arg0 ? 1 : 0);
		}

		@Override
		public Boolean convertReverse(String arg0) {
			return !model.getElementAt(0).equals(arg0); 
		}
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
    private JTextField txtTmcTpowerdown;
    private JTextField txtTmcPwmThrs;
    private JTextField txtTmcThigh;
    private JTextField txtTmcTstep;
    private JTextField txtTmcTcoolthrs;
    private JTextField txtTmcPwmconfAmpl;
    private JTextField txtTmcPwmscale;
    private JTextField txtTmcPwmconfGrad;
    private JTextField txtTmcDrvstatusSgresult;
    private JTextField txtTmcDrvstatusCsactual;
    private JTextField txtTmcIoinVersion;
    private JTextField txtTmcCoolconfSgt;
    private JCheckBox chkTmcGconfEnpwmmode;
    private JComboBox cmbTmcGconfSh;
    private JCheckBox chkTmcGstatReset;
    private JCheckBox chkTmcGstatDrvErr;
    private JCheckBox chkTmcGstatUvcp;
    private JComboBox cmbTmcIhrIhold;
    private JComboBox cmbTmcIhrIrun;
    private JComboBox cmbTmcIhrIholddelay;
    private JComboBox cmbTmcChopchonfToff;
    private JComboBox cmbTmcChopchonfHstrt;
    private JComboBox cmbTmcChopchonfHend;
    private JComboBox cmbTmcChopchonfFd3;
    private JComboBox cmbTmcChopchonfDisfdcc;
    private JComboBox cmbTmcChopchonfRndtf;
    private JComboBox cmbTmcChopchonfChm;
    private JComboBox cmbTmcChopchonfTbl;
    private JComboBox cmbTmcChopchonfVsense;
    private JComboBox cmbTmcChopchonfVhighfs;
    private JComboBox cmbTmcChopchonfVhighchm;
    private JComboBox cmbTmcChopchonfSync;
    private JComboBox cmbTmcChopchonfMres;
    private JComboBox cmbTmcChopchonfIntpol;
    private JComboBox cmbTmcChopchonfDedge;
    private JComboBox cmbTmcChopconfDiss2g;
    private JComboBox cmbTmcCoolconfSfilt;
    private JComboBox cmbTmcCoolconfSeimin;
    private JComboBox cmbTmcCoolconfSedn;
    private JComboBox cmbTmcCoolconfSemax;
    private JComboBox cmbTmcCoolconfSeup;
    private JComboBox cmbTmcCoolconfSemin;
    private JCheckBox chkTmcDrvstatusStst;
    private JCheckBox chkTmcDrvstatusOla;
    private JCheckBox chkTmcDrvstatusOlb;
    private JCheckBox chkTmcDrvstatusS2gb;
    private JCheckBox chkTmcDrvstatusS2ga;
    private JCheckBox chkTmcDrvstatusOtpw;
    private JCheckBox chkTmcDrvstatusOt;
    private JCheckBox chkTmcDrvstatusStallguard;
    private JCheckBox chkTmcDrvstatusFsactive;
    private JComboBox cmbTmcPwmconfFreq;
    private JCheckBox chkTmcPwmconfAutoscale;
    private JComboBox cmbTmcPwmconfSymmetric;
    private JComboBox cmbTmcPwmconfFreewheel;
}
