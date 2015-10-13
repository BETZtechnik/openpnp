package org.openpnp.spi.base;

import java.util.Collections;
import java.util.List;

import javax.swing.Icon;

import org.openpnp.model.Configuration;
import org.openpnp.spi.Actuator;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Head;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.PasteDispenser;
import org.openpnp.util.IdentifiableList;
import org.simpleframework.xml.Attribute;
import org.simpleframework.xml.ElementList;
import org.simpleframework.xml.core.Commit;

public abstract class AbstractHead implements Head {
    protected Machine machine;

    @Attribute
    protected String id;
    
    @Attribute(required=false)
    protected String name;
    
    @ElementList(required=false)
    protected IdentifiableList<Nozzle> nozzles = new IdentifiableList<Nozzle>();
    
    @ElementList(required=false)
    protected IdentifiableList<Actuator> actuators = new IdentifiableList<Actuator>();
    
    @ElementList(required=false)
    protected IdentifiableList<Camera> cameras = new IdentifiableList<Camera>();
    
    @ElementList(required=false)
    protected IdentifiableList<PasteDispenser> pasteDispensers = new IdentifiableList<PasteDispenser>();
    
    public AbstractHead() {
        this.id = Configuration.createId();
        this.name = getClass().getSimpleName();
    }
    
    @SuppressWarnings("unused")
    @Commit
    private void commit() {
        for (Nozzle nozzle : nozzles) {
            nozzle.setHead(this);
        }
        for (Camera camera : cameras) {
            camera.setHead(this);
        }
        for (Actuator actuator : actuators) {
            actuator.setHead(this);
        }
        for (PasteDispenser pasteDispenser : pasteDispensers) {
            pasteDispenser.setHead(this);
        }
    }
    
    @Override
    public String getId() {
        return id;
    }

    @Override
    public List<Nozzle> getNozzles() {
        return Collections.unmodifiableList(nozzles);
    }

    @Override
    public Nozzle getNozzle(String id) {
        return nozzles.get(id);
    }

    @Override
    public List<Actuator> getActuators() {
        return Collections.unmodifiableList(actuators);
    }

    @Override
    public Actuator getActuator(String id) {
        return actuators.get(id);
    }
    
    @Override
    public Actuator getActuatorByName(String name) {
        for (Actuator actuator : actuators) {
            if (actuator.getName().equals(name)) {
                return actuator;
            }
        }
        return null;
    }

    @Override
    public List<Camera> getCameras() {
        return Collections.unmodifiableList(cameras);
    }

    @Override
    public Camera getCamera(String id) {
        return cameras.get(id);
    }

    @Override
    public void addCamera(Camera camera) throws Exception {
        cameras.add(camera);
    }

    @Override
    public void removeCamera(Camera camera) {
        cameras.remove(camera);
    }
    
    @Override
    public void moveToSafeZ(double speed) throws Exception {
        for (Nozzle nozzle : nozzles) {
            nozzle.moveToSafeZ(speed);
        }
        for (Camera camera : cameras) {
            camera.moveToSafeZ(speed);
        }
        for (Actuator actuator : actuators) {
            actuator.moveToSafeZ(speed);
        }
        for (PasteDispenser dispenser : pasteDispensers) {
            dispenser.moveToSafeZ(speed);
        }
    }
    
    @Override
    public String getName() {
        return name;
    }

    @Override
    public void setName(String name) {
        this.name = name;
    }   
    
    @Override
    public List<PasteDispenser> getPasteDispensers() {
        return Collections.unmodifiableList(pasteDispensers);
    }

    @Override
    public PasteDispenser getPasteDispenser(String id) {
        return pasteDispensers.get(id);
    }

    @Override
    public Icon getPropertySheetHolderIcon() {
        // TODO Auto-generated method stub
        return null;
    }

	@Override
	public Camera getDefaultCamera() {
		List<Camera> cameras = getCameras();
		if (cameras == null || cameras.isEmpty()) {
			return null;
		}
		return cameras.get(0);
	}

	@Override
	public Nozzle getDefaultNozzle() {
		List<Nozzle> nozzles = getNozzles();
		if (nozzles == null || nozzles.isEmpty()) {
			return null;
		}
		return nozzles.get(0);
	}

	@Override
	public PasteDispenser getDefaultPasteDispenser() {
		List<PasteDispenser> dispensers = getPasteDispensers();
		if (dispensers == null || dispensers.isEmpty()) {
			return null;
		}
		return dispensers.get(0);
	}
}
