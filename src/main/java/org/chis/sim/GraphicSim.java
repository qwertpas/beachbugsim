package org.chis.sim;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Toolkit;
import java.awt.image.BufferedImage;
import java.awt.geom.AffineTransform;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;

import org.chis.sim.math.*;
import org.chis.sim.math.Vector2D.Type;
import org.chis.sim.wheels.CoaxSwerveModule;
import org.chis.sim.wheels.Wheel;

//draws the robot
public class GraphicSim extends JPanel {

	static JFrame frame;

	static BufferedImage robotImage;
	static BufferedImage coaxModuleImage;
	static BufferedImage diffModuleImage;
	static BufferedImage omniWheelImage;
	static BufferedImage wheelImage;

	static int screenHeight, screenWidth;
	static int windowWidth, windowHeight;
	static GraphicSim sim;

	static double DISP_SCALE;

	static AffineTransform before;

	public static String imagesDirectory = "./src/images/";

	public static List<Serie> userPointsRobot = Collections.synchronizedList(new ArrayList<Serie>());
	public static List<Serie> userPointsGlobal = Collections.synchronizedList(new ArrayList<Serie>());

	public static void init(){
		screenWidth = (int) Toolkit.getDefaultToolkit().getScreenSize().getWidth();
		screenHeight = (int) Toolkit.getDefaultToolkit().getScreenSize().getHeight();
		try {
			robotImage = ImageIO.read(new File(imagesDirectory, "robot.png"));
			coaxModuleImage = ImageIO.read(new File(imagesDirectory, "coaxModule.png"));
			diffModuleImage = ImageIO.read(new File(imagesDirectory, "diffModule.png"));
			omniWheelImage = ImageIO.read(new File(imagesDirectory, "omniWheel.png"));
			wheelImage = ImageIO.read(new File(imagesDirectory, "wheel.png"));
		} catch (IOException e) {
			e.printStackTrace();
		}
		frame = new JFrame("Robot Sim");
		sim = new GraphicSim();
		frame.add(sim);
		frame.setSize(screenWidth - 200, screenHeight);
		frame.setLocation(200, 0);
		frame.setVisible(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		updateConstants();
	}

	public static void updateConstants(){
		DISP_SCALE = Constants.DISPLAY_SCALE.getDouble();
	}

    @Override
	public void paint(Graphics g) { //gets called iteratively by JFrame
		super.paint(g);
		Graphics2D g2d = (Graphics2D) g;

		windowWidth = (int) g.getClipBounds().getWidth();
		windowHeight = (int) g.getClipBounds().getHeight();


		//center the grid and flip y so it is up (default y is down)
		g2d.translate(windowWidth/2, windowHeight/2);
		g2d.scale(1, -1);

		//draw vertical lines
		for(int ix = 0; ix < windowWidth/2; ix += (int) DISP_SCALE){
			if(ix == 0){
				g.setColor(Color.GRAY.darker());
			}else{
				g.setColor(Color.GRAY.brighter());
			}
			g.drawLine(ix, -windowHeight/2, ix, windowHeight/2);
			g.drawLine(-ix, -windowHeight/2, -ix, windowHeight/2);
		}

		//draw horizontal lines
		for(int iy = 0; iy < windowHeight/2; iy += (int) DISP_SCALE){
			if(iy == 0){
				g.setColor(Color.GRAY.darker());
			}else{
				g.setColor(Color.GRAY.brighter());
			}
			g.drawLine(-windowWidth/2, iy, windowWidth/2, iy);
			g.drawLine(-windowWidth/2, -iy, windowWidth/2, -iy);
		}

		//draw global points
		synchronized(userPointsGlobal){
			for(Serie serie : userPointsGlobal){
				g.setColor(serie.color);
				for(int i = 0; i < serie.points.size() - 1; i++){
					int[] scaledPos1 = meterToPixel(serie.points.get(i).x, serie.points.get(i).y, false);
					int[] scaledPos2 = meterToPixel(serie.points.get(i + 1).x, serie.points.get(i + 1).y, false);
					g.drawLine(scaledPos1[0], scaledPos1[1], scaledPos2[0], scaledPos2[1]);
				}
			}
		}
			
			
		//robot transform in pixels
		int[] robotPixelPos = meterToPixel(Main.robot.robotPos.x, Main.robot.robotPos.y, true);
		g2d.translate(robotPixelPos[0], robotPixelPos[1]);
		g2d.rotate(Main.robot.robotPos.ang);

		//scaling down to draw robot and then scaling back up
        // g2d.scale(robotScale, robotScale);
		// g.drawImage(robotImage, -robotImage.getWidth()/2, -robotImage.getHeight()/2, this);
		// g2d.scale(1/robotScale, 1/robotScale);

		double robotWidthReal = 0;
		double robotLengthReal = 0;
		for(Wheel wheel : Main.robot.wheels){
			double xDist = 2 * (Math.abs(wheel.placement.x) + 2 * wheel.wheelRadius);
			double yDist = 2 * (Math.abs(wheel.placement.y) + 2 * wheel.wheelRadius);
			if(xDist > robotWidthReal){
				robotWidthReal = xDist;
			}
			if(yDist > robotLengthReal){
				robotLengthReal = yDist;
			}
		}

		int robotWidthDisplay = (int) (DISP_SCALE * robotWidthReal); //width of robot in pixels
		int robotLengthDisplay = (int) (DISP_SCALE * robotLengthReal); //length of robot in pixels

		g.setColor(Color.BLACK);
		g2d.fillRect(-robotWidthDisplay / 2, -robotLengthDisplay / 2, robotWidthDisplay, robotLengthDisplay);

		g.setColor(Color.white);
		g2d.fillRect(robotLengthDisplay/2 - 10, -5, 10, 10);
		
		g.setColor(Color.RED);
		for(Wheel wheel : Main.robot.wheels){
			switch(wheel.wheelType){
				case CoaxSwerveModule:
					CoaxSwerveModule coax = (CoaxSwerveModule) wheel;

					drawCentered(coaxModuleImage, coax.placement, coax.wheelRadius * 3, g2d);
					drawCentered(wheelImage, coax.placement.rotateAng(coax.wheelTurnIntegrator.pos), coax.wheelRadius * 2.8, g2d);

					drawForce(coax.placement, new Vector2D(coax.driveMotor.torque, coax.placement.ang + coax.wheelTurnIntegrator.pos, Type.POLAR), 10, g2d);
					// g2d.drawImage(wheelImage, 100, 100, this);
					break;
				case DiffSwerveModule:
					drawCentered(coaxModuleImage, wheel.placement, wheel.wheelRadius * 2, g2d);
					break;
				case FixedWheel:
					g.drawImage(robotImage, -robotImage.getWidth()/2, -robotImage.getHeight()/2, this);
					break;
				case OmniWheel:
					g.drawImage(robotImage, -robotImage.getWidth()/2, -robotImage.getHeight()/2, this);
					break;
				default:
					break;
			}
		}

		//draw robot relative points
		synchronized(userPointsRobot){
			for(Serie serie : userPointsRobot){
				g.setColor(serie.color);
				for(int i = 0; i < serie.points.size() - 1; i++){
					int[] scaledPos1 = meterToPixel(serie.points.get(i).x, serie.points.get(i).y, false);
					int[] scaledPos2 = meterToPixel(serie.points.get(i + 1).x, serie.points.get(i + 1).y, false);
					g.drawLine(scaledPos1[0], scaledPos1[1], scaledPos2[0], scaledPos2[1]);
				}
			}
		}
			

	}

	public void drawCentered(BufferedImage img, Pose2D offset, double realWidth, Graphics2D g2d){
		int displayWidth = (int) (DISP_SCALE * realWidth); //in pixels
		double scale = (double) displayWidth / img.getWidth();

		g2d.translate(DISP_SCALE * offset.x, DISP_SCALE * offset.y);
		g2d.scale(scale, scale);
		g2d.rotate(offset.ang);

		g2d.drawImage(img, -img.getWidth()/2, -img.getWidth()/2, this);

		g2d.rotate(-offset.ang);
		g2d.scale(1/scale, 1/scale);
		g2d.translate(-DISP_SCALE * offset.x, -DISP_SCALE * offset.y);
	}

	public void drawForce(Pose2D offset, Vector2D force, double scale, Graphics2D g2d){
		g2d.translate(DISP_SCALE * offset.x, DISP_SCALE * offset.y);
		g2d.rotate(offset.ang);

		g2d.drawLine(0, 0, (int) (force.x * scale), (int) (force.y * scale));

		g2d.rotate(-offset.ang);
		g2d.translate(-DISP_SCALE * offset.x, -DISP_SCALE * offset.y);
	}

	
    public int[] meterToPixel(double xMeters, double yMeters, boolean loopback){
        int pixelX = (int) (xMeters * DISP_SCALE);
		int pixelY = (int) (yMeters * DISP_SCALE);
		
		if(loopback){
			pixelX = (int) Util.centerModulo(pixelX, windowWidth/2);
			pixelY = (int) Util.centerModulo(pixelY, windowHeight/2);
		}

        return new int[] {pixelX, pixelY};
    }

	public static void addDrawingRobot(ArrayList<Vector2D> path, Color color){
		synchronized(userPointsRobot){
			userPointsRobot.add(new Serie("robotDrawing", color, path));
		}
	}

	public static void addDrawingGlobal(ArrayList<Vector2D> path, Color color){
		synchronized(userPointsGlobal){
			userPointsGlobal.add(new Serie("globalDrawing", color, path));
		}	
	}
	
	public static void clearDrawing(){
		userPointsGlobal.clear();
		userPointsRobot.clear();
	}

	public static class PathPlotter{
		
	}


	//JPanel requires this, idk why
	private static final long serialVersionUID = -87884863222799400L;
}