package compsys704;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.util.Scanner;
import com.fazecast.jSerialComm.*;
import java.io.*;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import java.awt.Color;
import java.awt.Font;


public class Localization {
	static BufferedImage image;
	static BufferedImage overlay1;
	static JLabel label;
	static String mode = "y";
	static InputStream is;
	static SerialPort serial;
	static int x = 0;
	static int y = 0;
	static double meterX=5;
	static double meterY=5;

	public static void main(String[] args) {
		JFrame frame = new JFrame();
		int i = 1;
		startSerial();

		while (true) {
			serial();
			writeToFile();
			drawFinal();
			try {
				image = ImageIO.read(new File("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\combined.png"));
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			if (i > 1) {
				frame.getContentPane().removeAll();
			}
			frame.getContentPane().add(new JLabel(new ImageIcon(image)));
			frame.pack();
			frame.setVisible(true);
			i++;
		}

	}

	public static void drawFinal() {
		BufferedImage node;
		BufferedImage origin;
		try {
			// load source images

			image = ImageIO.read(new File("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\room.gif"));
			overlay1 = ImageIO.read(new File("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\robot.png"));
			origin = ImageIO.read(new File("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\origin.png"));
			node = ImageIO.read(new File("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\node.png"));

			// finish = ImageIO.read(new File(""));
			// create the new image, canvas size is the max. of both image sizes
			int w = image.getWidth();
			int h = image.getHeight();
			// System.out.println(w+" "+h);
			BufferedImage combined = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);

			// paint both images, preserving the alpha channels
			Graphics g = combined.getGraphics();
			double node1X, node1Y, node2X, node2Y, node3X, node3Y, node4X, node4Y;

			node1X = 0;
			node1Y = 2.5;
			node2X = 0;
			node2Y = 14.5;
			node3X = 10.6;
			node3Y = 11.4;
			node4X = 10.6;
			node4Y = 1.4;

			g.drawImage(image, 0, 0, null);
			g.drawImage(origin, -10, 520, null);
			int pixNode1X = (int) meterToPixels(node1X, 'x');
			int pixNode1Y = (int) meterToPixels(node1Y, 'y');
			g.drawImage(node, pixNode1X, pixNode1Y, null);
			int pixNode2X = (int) meterToPixels(node2X, 'x');
			int pixNode2Y = (int) meterToPixels(node2Y, 'y');
			g.drawImage(node, pixNode2X, pixNode2Y, null);
			int pixNode3X = (int) meterToPixels(node3X, 'x');
			int pixNode3Y = (int) meterToPixels(node3Y, 'y');
			g.drawImage(node, pixNode3X, pixNode3Y, null);
			int pixNode4X = (int) meterToPixels(node4X, 'x');
			int pixNode4Y = (int) meterToPixels(node4Y, 'y');
			g.drawImage(node, pixNode4X, pixNode4Y, null);
			g.setColor(Color.BLACK);
			g.setFont(new Font("Arial Black", Font.BOLD, 20));
			g.drawString("x:" + meterX + "m" + "  y:" + meterY + "m", 190, 610);

			int robotX = (int) meterToPixels(meterX, 'x');
			int robotY = (int) meterToPixels(meterY, 'y');
			g.drawImage(overlay1, robotX, robotY, null);

			ImageIO.write(combined, "PNG",
					new File("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\combined.png"));

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void serial() {

		is = serial.getInputStream();
		int in;
		int i = 0;
		try {
			
			boolean xNegitive =false;
			boolean yNegitive =false;
			while (true) {
			
				in = is.read();
				
				if (in == '*') {
					System.out.println("start of packet");
					i = 0;
				}

				in = in - 48;
				System.out.println(in+" "+i);
				if((in==-3)&&(i==1)){
					in = is.read();
					in = in - 48;
					xNegitive=true;
					System.out.println("x is negitive");
				}else if((in==-3)&&(i==6)){
					in = is.read();
					in = in - 48;
					yNegitive=true;
					System.out.println("y is negitive");
				}
				if (i == 1) {
					meterX = in * 10;
				} else if (i == 2) {
					meterX = meterX + in;
				} else if (i == 3) {
					meterX = meterX + in * .1;
				} else if (i == 4) {
					meterX = meterX + in * .01;
				} else if (i == 5) {
					meterX = meterX+in * .001;
				} else if (i == 6) {
					
					meterY =  in*10;
				} else if (i == 7) {
					meterY = meterY + in;
				} else if (i == 8) {
					meterY = meterY + in * .1;
				} else if (i == 9) {
					meterY = meterY + in * .01;
				} else if (i == 10) {
					meterY = meterY + in * .001;
					break;
				}
				i++;
			}
			if(xNegitive==true){
				meterX=meterX*-1;
			}
			if(yNegitive==true){
				meterY=meterY*-1;
			}
			
			meterX=meterX*1000;
			meterY=meterY*1000;
			meterX = Math.round(meterX);
			meterY = Math.round(meterY);
			meterX=meterX/1000;
			meterY=meterY/1000;
			System.out.println("finished reading position");
			System.out.println("x:"+meterX);
			System.out.println("y:"+meterY);

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public static void startSerial() {
		try {

			serial = SerialPort.getCommPort("COM3");
			boolean testPort = serial.openPort();
			if (testPort) {
				System.out.println("Comm port openned");
			} else {
				System.out.println("Failed");
			}

		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static double meterToPixels(double m, char type) {
		if (type == 'x') {
			m = m * 40;
		} else if (type == 'y') {
			m = m * 35;
			m = (m - 520) * (-1);
			// System.out.println(m);
		}

		return m;
	}
	
	public static void writeToFile() {
		
		String positionS = "X:"+meterX+" Y:"+meterY;
		try { 
			
			//writer = new PrintWriter("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\history.txt", "UTF-8");
			
			BufferedWriter output = new BufferedWriter(new FileWriter("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\history.txt", true));
			output.append(positionS);
			output.newLine();
			output.close();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

}
