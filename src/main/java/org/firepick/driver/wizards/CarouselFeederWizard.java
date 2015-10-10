package org.firepick.driver.wizards;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import org.firepick.feeder.CarouselFeeder;
import org.openpnp.gui.MainFrame;
import org.openpnp.gui.components.ComponentDecorators;
import org.openpnp.gui.support.DoubleConverter;
import org.openpnp.gui.support.IntegerConverter;
import org.openpnp.gui.support.MessageBoxes;
import org.openpnp.machine.reference.feeder.wizards.AbstractReferenceFeederConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Head;
import org.openpnp.spi.Nozzle;
import org.openpnp.util.MovableUtils;

import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.FormSpecs;
import com.jgoodies.forms.layout.RowSpec;

public class CarouselFeederWizard extends
        AbstractReferenceFeederConfigurationWizard {
    final private CarouselFeeder feeder;
    private JTextField carouselAddressTextField;
    private JTextField trayNumberTextField;
    private JTextField partVisionWidthTextField;
    private JTextField partVisionHeightTextField;
    
    public CarouselFeederWizard(CarouselFeeder feeder) {
        super(feeder);
        this.feeder = feeder;
        
        JPanel settings = new JPanel();
        contentPanel.add(settings);
        settings.setLayout(new FormLayout(new ColumnSpec[] {
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
                FormSpecs.DEFAULT_ROWSPEC,}));
        
        JLabel lblCarouselAddress = new JLabel("Carousel Address");
        settings.add(lblCarouselAddress, "2, 2, right, default");
        
        carouselAddressTextField = new JTextField();
        settings.add(carouselAddressTextField, "4, 2, fill, default");
        carouselAddressTextField.setColumns(10);
        
        JLabel lblTrayNumber = new JLabel("Tray Number");
        settings.add(lblTrayNumber, "2, 4, right, default");
        
        trayNumberTextField = new JTextField();
        settings.add(trayNumberTextField, "4, 4, fill, default");
        trayNumberTextField.setColumns(10);
        
        JLabel lblPartVisionWidth = new JLabel("Part Vision Width");
        settings.add(lblPartVisionWidth, "2, 6, right, default");
        
        partVisionWidthTextField = new JTextField();
        settings.add(partVisionWidthTextField, "4, 6, fill, default");
        partVisionWidthTextField.setColumns(10);
        
        JLabel lblPartVisionHeight = new JLabel("Part Vision Height");
        settings.add(lblPartVisionHeight, "2, 8, right, default");
        
        partVisionHeightTextField = new JTextField();
        settings.add(partVisionHeightTextField, "4, 8, fill, default");
        partVisionHeightTextField.setColumns(10);
        
        JButton btnTrain = new JButton(train);
        settings.add(btnTrain, "4, 10");
    }    
    
    @Override
    public void createBindings() {
        super.createBindings();
        IntegerConverter intConverter = new IntegerConverter();
        DoubleConverter doubleConverter = new DoubleConverter(Configuration.get().getLengthDisplayFormat());

        addWrappedBinding(feeder, "address", carouselAddressTextField, "text", intConverter);
        addWrappedBinding(feeder, "tray", trayNumberTextField, "text", intConverter);
        addWrappedBinding(feeder, "partVisionWidth", partVisionWidthTextField, "text", doubleConverter);
        addWrappedBinding(feeder, "partVisionHeight", partVisionHeightTextField, "text", doubleConverter);

        ComponentDecorators.decorateWithAutoSelect(carouselAddressTextField);
        ComponentDecorators.decorateWithAutoSelect(trayNumberTextField);
        ComponentDecorators.decorateWithAutoSelect(partVisionWidthTextField);
        ComponentDecorators.decorateWithAutoSelect(partVisionHeightTextField);
    }
    
    /*
     * Training process:
     * instruction: place a single part in the tray
     * instruction: touch the nozzle to the part
     * machine: move camera to nozzle position
     * machine: capture closest rect to nozzle
     * machine: determine rect width, height, angle and draw it on the overlay
     * for the user to see.
     * instruction: confirm that the part is positioned correctly 
     */
    @SuppressWarnings("serial")
    private Action train = new AbstractAction(
            "Train") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                feeder.selectTray();
                MainFrame.mainFrame.showInstructions(
                        "Carousel Vision Training", 
                        "Place a single part in the tray.", 
                        true, 
                        true, 
                        "Next", 
                        trainCancel, 
                        trainPartPlaced);
            }
            catch (Exception e) {
                MessageBoxes.errorBox(CarouselFeederWizard.this, "Error", e);
            }
        }
    };
    
    @SuppressWarnings("serial")
    private Action trainPartPlaced = new AbstractAction(
            "Cancel") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            MainFrame.mainFrame.showInstructions(
                    "Carousel Vision Training", 
                    "Jog the nozzle so it is touching the part.", 
                    true, 
                    true, 
                    "Next", 
                    trainCancel, 
                    trainPartTouched);
        }
    };

    @SuppressWarnings("serial")
    private Action trainPartTouched = new AbstractAction(
            "Cancel") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                Head head = Configuration.get().getMachine().getHeads().get(0);
                Nozzle nozzle = head.getNozzles().get(0);
                Camera camera = head.getCameras().get(0);
                MovableUtils.moveToLocationAtSafeZ(camera, nozzle.getLocation(), 1.0);
                
                MainFrame.mainFrame.showInstructions(
                        "Carousel Vision Training", 
                        "Jog the nozzle so it is touching the part.", 
                        true, 
                        true, 
                        "Next", 
                        trainCancel, 
                        trainPartTouched);
            }
            catch (Exception e) {
                MessageBoxes.errorBox(CarouselFeederWizard.this, "Error", e);
            }
        }
    };

    @SuppressWarnings("serial")
    private Action trainCancel = new AbstractAction(
            "Cancel") {
        @Override
        public void actionPerformed(ActionEvent arg0) {
            MainFrame.mainFrame.hideInstructions();
        }
    };
}
