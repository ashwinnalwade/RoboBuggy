package pointsTracker;

import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.IOException;

import javax.swing.ButtonGroup;
import javax.swing.JComponent;
import javax.swing.JRadioButton;

public class Display extends JComponent implements ActionListener{
	
	GraphPanel mapCover;

	// Note that a final field can be initialized in constructor
	private final int DISPLAY_WIDTH;   
	private final int DISPLAY_HEIGHT;


	public Display(int width, int height) {
		DISPLAY_WIDTH = width;
		DISPLAY_HEIGHT = height;
		init();
	}


	public void init() {
		mapCover = new GraphPanel(DISPLAY_WIDTH, DISPLAY_HEIGHT);
		mapCover.setBounds(0, 60, mapCover.WIDE, mapCover.HIGH);
		setSize(DISPLAY_WIDTH, DISPLAY_HEIGHT);
		initRadioGroup();
		add(mapCover);
		
		
		repaint();
	}


	private void initRadioGroup() {
		// TODO Auto-generated method stub
		JRadioButton startButton = new JRadioButton("Start");
		startButton.setBounds(0, 10, 100, 20);
		startButton.setActionCommand("start");

	    JRadioButton midButton = new JRadioButton("Middle");
	    midButton.setBounds(100, 10, 100, 20);
	    midButton.setActionCommand("mid");

	    JRadioButton endButton = new JRadioButton("End");
	    endButton.setBounds(200, 10, 100, 20);
	    endButton.setActionCommand("end");
	    
	    JRadioButton noneButton = new JRadioButton("None");
	    noneButton.setBounds(300, 10, 100, 20);
	    noneButton.setActionCommand("none");

	    //Group the radio buttons.
	    ButtonGroup group = new ButtonGroup();
	    group.add(startButton);
	    group.add(midButton);
	    group.add(endButton);
	    group.add(noneButton);
	    
	    add(startButton);
	    add(midButton);
	    add(endButton);
	    add(noneButton);
	    

	    //Register a listener for the radio buttons.
	    startButton.addActionListener(this);
	    midButton.addActionListener(this);
	    endButton.addActionListener(this);
	    noneButton.addActionListener(this);
	}


	public void paintComponent(Graphics g) {
		mapCover.repaint();
	}


	@Override
	public void actionPerformed(ActionEvent arg0) {
		// TODO Auto-generated method stub
		System.out.println(arg0.getActionCommand());
		if(arg0.getActionCommand().equals("start")) mapCover.setPinMode(GraphPanel.PIN_MODE_START);
		else if(arg0.getActionCommand().equals("mid")) mapCover.setPinMode(GraphPanel.PIN_MODE_MID);
		else if(arg0.getActionCommand().equals("end")) mapCover.setPinMode(GraphPanel.PIN_MODE_END);
		else mapCover.setPinMode(GraphPanel.PIN_MODE_OOPS);
		
	}
	
	
    


}