package compsys704;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.util.Scanner;
import com.fazecast.jSerialComm.*;
import java.io.*;
import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

public class Localization {
	static BufferedImage image;
	static BufferedImage overlay1;
	static JLabel label;
	static String mode = "y";
	static InputStream is;
	static SerialPort serial;
	static int x=0;
	static int y=0;
	public static void main(String[] args) {
		JFrame frame = new JFrame();
		int i = 1;
		startSerial();
		

		while (true) {
			serial();
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

		BufferedImage finish;
		try {
			// load source images

			image = ImageIO.read(new File("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\grid room.PNG"));
			overlay1 = ImageIO.read(new File("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\robot.png"));
			finish = ImageIO.read(new File("C:\\Users\\User\\workspace\\compsys704\\src\\compsys704\\finish.png"));

			// finish = ImageIO.read(new File(""));
			// create the new image, canvas size is the max. of both image sizes
			int w = image.getWidth();
			int h = image.getHeight();
			// System.out.println(w+" "+h);
			BufferedImage combined = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);

			// paint both images, preserving the alpha channels
			Graphics g = combined.getGraphics();
			g.drawImage(image, 0, 0, null);
			g.drawImage(finish, 300, 235, null);
			Scanner reader = new Scanner(System.in); // Reading from System.in
//			System.out.println("Enter x: ");
//			int x = reader.nextInt();
//			System.out.println("Enter y: ");
//			int y = reader.nextInt();
			g.drawImage(overlay1, x, y, null);

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
			try {
				in = is.read();
				in = in-48;
				x=in;
				System.out.println(in);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		
	}

	public static void startSerial() {
		try {

			serial = SerialPort.getCommPort("COM4");
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

}
