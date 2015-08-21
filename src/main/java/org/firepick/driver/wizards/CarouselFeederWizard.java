package org.firepick.driver.wizards;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import org.firepick.delta.CarouselFeeder;
import org.openpnp.gui.components.ComponentDecorators;
import org.openpnp.gui.support.DoubleConverter;
import org.openpnp.gui.support.IntegerConverter;
import org.openpnp.machine.reference.feeder.wizards.AbstractReferenceFeederConfigurationWizard;
import org.openpnp.model.Configuration;

import com.jgoodies.forms.factories.FormFactory;
import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.RowSpec;

public class CarouselFeederWizard extends
        AbstractReferenceFeederConfigurationWizard {
    final private CarouselFeeder feeder;
    private JTextField carouselAddressTextField;
    private JTextField trayNumberTextField;
    private JTextField partAreaTextField;
    
    public CarouselFeederWizard(CarouselFeeder feeder) {
        super(feeder);
        this.feeder = feeder;
        
        JPanel settings = new JPanel();
        contentPanel.add(settings);
        settings.setLayout(new FormLayout(new ColumnSpec[] {
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
        
        JLabel lblPartArea = new JLabel("Part Area");
        settings.add(lblPartArea, "2, 6, right, default");
        
        partAreaTextField = new JTextField();
        settings.add(partAreaTextField, "4, 6, fill, default");
        partAreaTextField.setColumns(10);
    }    
    
    @Override
    public void createBindings() {
        super.createBindings();
        IntegerConverter intConverter = new IntegerConverter();
        DoubleConverter doubleConverter = new DoubleConverter(Configuration.get().getLengthDisplayFormat());

        addWrappedBinding(feeder, "address", carouselAddressTextField, "text", intConverter);
        addWrappedBinding(feeder, "tray", trayNumberTextField, "text", intConverter);
        addWrappedBinding(feeder, "partArea", partAreaTextField, "text", doubleConverter);

        ComponentDecorators.decorateWithAutoSelect(carouselAddressTextField);
        ComponentDecorators.decorateWithAutoSelect(trayNumberTextField);
        ComponentDecorators.decorateWithAutoSelect(partAreaTextField);
    }
}
