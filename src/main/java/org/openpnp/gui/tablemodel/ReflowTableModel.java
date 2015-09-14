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

package org.openpnp.gui.tablemodel;

import javax.swing.table.AbstractTableModel;

import org.openpnp.model.Board;
import org.openpnp.model.Configuration;
import org.openpnp.model.ReflowProfileStep;

public class ReflowTableModel extends AbstractTableModel {
	final Configuration configuration;
	
	private String[] columnNames = new String[] {
	    "Name",
        "Max. Rate (°C/sec)", 
		"Target (°C)", 
        "Hold Time (sec)" 
		};
	
	private Class[] columnTypes = new Class[] {
		String.class,
        String.class,
        Double.class,
		Integer.class
	};
	
	private Board board;

	public ReflowTableModel(Configuration configuration) {
		this.configuration = configuration;
	}

	public void setBoard(Board board) {
		this.board = board;
		fireTableDataChanged();
	}

	@Override
	public String getColumnName(int column) {
		return columnNames[column];
	}

	public int getColumnCount() {
		return columnNames.length;
	}

	public int getRowCount() {
		return (board == null) ? 0 : board.getReflowProfile().size();
	}
	
	@Override
	public boolean isCellEditable(int rowIndex, int columnIndex) {
	    return true;
	}
	
	@Override
	public Class<?> getColumnClass(int columnIndex) {
		return columnTypes[columnIndex];
	}

	@Override
	public void setValueAt(Object aValue, int rowIndex, int columnIndex) {
		try {
			ReflowProfileStep step = board.getReflowProfile().get(rowIndex);
            if (columnIndex == 0) {
                step.setName((String) aValue);
            }
            else if (columnIndex == 1) {
                String s = (String) aValue;
                double v;
                try {
                    v = Double.parseDouble(s);
                }
                catch (Exception e) {
                    v = 0;
                }
                step.setMaxRateCelsiusPerSecond(v);
            }
            else if (columnIndex == 2) {
                step.setTargetTemperatureCelsius((Double) aValue);
            }
            else if (columnIndex == 3) {
                step.setHoldTimeSeconds((Integer) aValue);
            }
		}
		catch (Exception e) {
			// TODO: dialog, bad input
		}
	}
	
	public Object getValueAt(int row, int col) {
		ReflowProfileStep step = board.getReflowProfile().get(row);
		switch (col) {
	    case 0:
	        return step.getName();
        case 1:
            return step.getMaxRateCelsiusPerSecond() == 0 ? "Unlimited" : step.getMaxRateCelsiusPerSecond();
		case 2:
			return step.getTargetTemperatureCelsius();
		case 3:
			return step.getHoldTimeSeconds();
		default:
			return null;
		}
	}
}