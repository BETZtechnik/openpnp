package org.openpnp.gui;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JButton;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JToolBar;
import javax.swing.ListSelectionModel;
import javax.swing.SwingUtilities;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;

import org.openpnp.JobProcessorListener;
import org.openpnp.gui.components.AutoSelectTextTable;
import org.openpnp.gui.support.ActionGroup;
import org.openpnp.gui.support.Helpers;
import org.openpnp.gui.support.Icons;
import org.openpnp.gui.support.MessageBoxes;
import org.openpnp.gui.tablemodel.ReflowTableModel;
import org.openpnp.model.BoardLocation;
import org.openpnp.model.Configuration;
import org.openpnp.model.ReflowProfileStep;

import javax.swing.JLabel;

import java.awt.Font;
import java.awt.Component;

import javax.swing.Box;

import java.awt.Color;

public class JobReflowPanel extends JPanel {
    private JTable table;
    private ReflowTableModel tableModel;
    private ActionGroup boardLocationSelectionActionGroup;
    private ActionGroup singleSelectionActionGroup;
    private ActionGroup multiSelectionActionGroup;
    private BoardLocation boardLocation;

    public JobReflowPanel(JobPanel jobPanel) {
        Configuration configuration = Configuration.get();
        
        boardLocationSelectionActionGroup = new ActionGroup(newAction);
        boardLocationSelectionActionGroup.setEnabled(false);

        singleSelectionActionGroup = new ActionGroup(removeAction);
        singleSelectionActionGroup.setEnabled(false);

        multiSelectionActionGroup = new ActionGroup(removeAction);
        multiSelectionActionGroup.setEnabled(false);

        setLayout(new BorderLayout(0, 0));
        JToolBar toolBar = new JToolBar();
        add(toolBar, BorderLayout.NORTH);

        toolBar.setFloatable(false);
        JButton btnNew = new JButton(newAction);
        btnNew.setHideActionText(true);
        toolBar.add(btnNew);
        JButton btnRemove = new JButton(removeAction);
        btnRemove.setHideActionText(true);
        toolBar.add(btnRemove);
        toolBar.addSeparator();
        
        lblCurrentTemp = new JLabel("0 ºC");
        lblCurrentTemp.setForeground(new Color(0, 128, 0));
        lblCurrentTemp.setFont(new Font("Lucida Grande", Font.BOLD, 20));
        toolBar.add(lblCurrentTemp);
        
        Component horizontalStrut = Box.createHorizontalStrut(20);
        toolBar.add(horizontalStrut);
        
        lblTargetTemp = new JLabel("0 ºC");
        lblTargetTemp.setForeground(new Color(139, 0, 0));
        lblTargetTemp.setFont(new Font("Lucida Grande", Font.BOLD, 20));
        toolBar.add(lblTargetTemp);
        
        Component horizontalStrut_1 = Box.createHorizontalStrut(20);
        toolBar.add(horizontalStrut_1);
        
        lblHoldSecondsRemaining = new JLabel("0s");
        lblHoldSecondsRemaining.setForeground(new Color(0, 0, 139));
        lblHoldSecondsRemaining.setFont(new Font("Lucida Grande", Font.BOLD, 20));
        toolBar.add(lblHoldSecondsRemaining);

        tableModel = new ReflowTableModel(configuration);

        table = new AutoSelectTextTable(tableModel);
        table.setAutoCreateRowSorter(true);
        table.setSelectionMode(ListSelectionModel.SINGLE_INTERVAL_SELECTION);
        
        table.getSelectionModel().addListSelectionListener(
                new ListSelectionListener() {
                    @Override
                    public void valueChanged(ListSelectionEvent e) {
                        if (e.getValueIsAdjusting()) {
                            return;
                        }
                        
                        if (getSelections().size() > 1) {
                            // multi select
                            singleSelectionActionGroup.setEnabled(false);
                            multiSelectionActionGroup.setEnabled(true);
                        }
                        else {
                            // single select, or no select
                            multiSelectionActionGroup.setEnabled(false);
                            singleSelectionActionGroup.setEnabled(getSelection() != null);
                        }
                    }
                });
        
        JScrollPane scrollPane = new JScrollPane(table);
        add(scrollPane, BorderLayout.CENTER);
    }

    public void setBoardLocation(BoardLocation boardLocation) {
        this.boardLocation = boardLocation;
        if (boardLocation == null) {
            tableModel.setBoard(null);
            boardLocationSelectionActionGroup.setEnabled(false);
        }
        else {
            tableModel.setBoard(boardLocation.getBoard());
            boardLocationSelectionActionGroup.setEnabled(true);
        }
    }

    public ReflowProfileStep getSelection() {
        List<ReflowProfileStep> selections = getSelections();
        if (selections.isEmpty()) {
            return null;
        }
        return selections.get(0);
    }
    
    public List<ReflowProfileStep> getSelections() {
        ArrayList<ReflowProfileStep> rows = new ArrayList<ReflowProfileStep>();
        if (boardLocation == null) {
            return rows;
        }
        int[] selectedRows = table.getSelectedRows();
        for (int selectedRow : selectedRows) {
            selectedRow = table.convertRowIndexToModel(selectedRow);
            rows.add(boardLocation.getBoard().getReflowProfile().get(selectedRow));
        }
        return rows;
    }

    public final Action newAction = new AbstractAction() {
        {
            putValue(SMALL_ICON, Icons.add);
            putValue(NAME, "New Reflow Profile Step");
            putValue(SHORT_DESCRIPTION,
                    "Create a new reflow profile step and add it to the profile.");
        }

        @Override
        public void actionPerformed(ActionEvent arg0) {
            try {
                boardLocation.getBoard().addReflowProfileStep(new ReflowProfileStep());
                tableModel.fireTableDataChanged();
                Helpers.selectLastTableRow(table);
            }
            catch (Exception e) {
                MessageBoxes.errorBox(
                        JOptionPane.getFrameForComponent(JobReflowPanel.this),
                        "New Error", e);
            }
        }
    };

    public final Action removeAction = new AbstractAction() {
        {
            putValue(SMALL_ICON, Icons.delete);
            putValue(NAME, "Remove Reflow Profile Step");
            putValue(SHORT_DESCRIPTION,
                    "Remove the currently selected reflow profile step.");
        }

        @Override
        public void actionPerformed(ActionEvent arg0) {
            for (ReflowProfileStep o : getSelections()) {
                boardLocation.getBoard().removeReflowProfileStep(o);
            }
            tableModel.fireTableDataChanged();
        }
    };
    
    public JobProcessorListener jobProcessorListener = new JobProcessorListener.Adapter() {
        @Override
        public void reflowProgress(final ReflowProfileStep step,
                final double currentTemperatureCelsius, 
                final int currentHoldSeconds) {
            SwingUtilities.invokeLater(new Runnable() {
                @Override
                public void run() {
                    lblCurrentTemp.setText(String.format("%.2f ºC", currentTemperatureCelsius));
                    lblTargetTemp.setText(String.format("%.2f ºC", step.getTargetTemperatureCelsius()));
                    lblHoldSecondsRemaining.setText(String.format("%d sec", step.getHoldTimeSeconds() - currentHoldSeconds));
                }
            });
        }
    };
    private JLabel lblHoldSecondsRemaining;
    private JLabel lblTargetTemp;
    private JLabel lblCurrentTemp;
}
