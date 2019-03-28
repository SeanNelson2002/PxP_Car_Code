import javax.swing.JFrame;
import java.awt.Font;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Canvas;
import javax.swing.JPanel;
import java.net.*;
import java.io.*;
public class PxP_Car_Graphics extends JFrame
{
   private static final int WIDTH = 800;
   private static final int HEIGHT = 800;
   private int Battery_Level = 100;
   public PxP_Car_Graphics()
   {
      super("PawXPaw 4x4 Car Analytics");
      setSize(WIDTH,HEIGHT);
      getContentPane().add(new Car_Graphics_Panel(Battery_Level));
      setVisible(true);
      setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   }
   public void Update_Battery(int Battery_LVL, Graphics window)
   {
	   Battery_Level = Battery_LVL;
	   Car_Graphics_Panel stuff = new Car_Graphics_Panel(Battery_Level);
	   stuff.update(window);
	   }
   public static void main(String args[])
   {
      PxP_Car_Graphics run = new PxP_Car_Graphics();
      Client runner = new Client("255.255.255.255",7777,window);
      runner.start(window);
   }
}
class Client
{	/*
	private Socket socket=null;
	private DataInputStream input=null;
	private String line="";
	public Client(String address, int port)
	{
		try
		{
			socket = new Socket(address,port);
			input = new DataInputStream(System.in);
		} catch(UnknownHostException u)
		{
			System.out.println(u);
		} catch(IOException i)
		{
			System.out.println(i);
		}
		String line = "";
		while (!line.equals("Over"))
		{
			try
			{
				line = input.readLine();
			}
			catch(IOException i)
			{
				System.out.println(i);
			}
		}
		try
		{
			input.close();
			socket.close();
		}
		catch(IOException i)
		{
			System.out.println(i);
		}
	}*/
	public Client(String address, int port,Graphics window){}
		public void start(Graphics window)
		{/*
			while(true){
				Client client = new Client("255.255.255.255",7777);
				double[] data = new double[21];
				String[] stringdata = line.split(" ");
				int array_slot = 0;
				for(String item:stringdata){
					data[array_slot]=Double.parseDouble(item);
					array_slot++;
				}
				PxP_Car_Graphics run = new PxP_Car_Graphics();
				run.Update_Battery(((int)(data[19])));
			}
			*/
			PxP_Car_Graphics run = new PxP_Car_Graphics();
			for(int x=100; x>0;x--){
				run.Update_Battery(x,window);
			}
		}	
}
class Car_Graphics_Panel extends JPanel
{
	private int Battery_Level = 100;
   public Car_Graphics_Panel(int New_Battery_Level)
   {
	  int Battery_Level = New_Battery_Level;
      setBackground(Color.WHITE);
      setVisible(true);
   }
   public void update(Graphics window)
   {
      paint(window);
   }
   public void paint(Graphics window)
   {
      window.setColor(Color.WHITE);
      window.fillRect(0,0,getWidth(),getHeight());
      window.setColor(Color.BLACK);
      window.setFont(new Font("TAHOMA",Font.BOLD,25));
      window.drawString("Paw X Paw",265,40);
      Car_Battery battery = new Car_Battery();
      battery.update(window,Battery_Level);}
   }
class Car_Battery
{
   private int battery_Level = 100;
   private static int battery_Shell_xPos = 20;
   private static int battery_Shell_yPos = 20;
   private static int battery_Shell_Width = 52;
   private static int battery_Shell_Height = 20;
   private static int battery_Shell_Cap_xPos = 72;
   private static int battery_Shell_Cap_yPos = 25;
   private static int battery_Shell_Cap_Width = 5;
   private static int battery_Shell_Cap_Height = 10;
   private static Color battery_Level_Low_Color = Color.RED;
   private static Color battery_Level_Medium_Color = Color.YELLOW;
   private static Color battery_Level_High_Color = Color.GREEN;
   private int battery_Level_Width = battery_Level/2+1;
   public void draw(Graphics window)
   {
      window.setColor(Color.BLACK);
      window.drawRect(battery_Shell_xPos,battery_Shell_yPos, battery_Shell_Width, battery_Shell_Height);
      if (battery_Level>=66)
      {
         window.setColor(battery_Level_High_Color);
         window.fillRect(battery_Shell_xPos+1,battery_Shell_yPos+1,battery_Level_Width,battery_Shell_Height-1);
      } else if (battery_Level<66)
      {
         if (battery_Level>=33)
         {
            window.setColor(battery_Level_Medium_Color);
            window.fillRect(battery_Shell_xPos+1,battery_Shell_yPos+1,battery_Level_Width,battery_Shell_Height-1);
         } else {
            window.setColor(battery_Level_Low_Color);
            window.fillRect(battery_Shell_xPos+1,battery_Shell_yPos+1,battery_Level_Width,battery_Shell_Height-1);
         }
      }
      window.setColor(Color.BLACK);
      window.fillRect(battery_Shell_Cap_xPos,battery_Shell_Cap_yPos,battery_Shell_Cap_Width,battery_Shell_Cap_Height);
      window.setColor(Color.BLACK);
      window.setFont(new Font("TAHOMA",Font.BOLD,12));
      window.drawString(battery_Level+"%",30, 35);
   }
   public void update(Graphics window, int level)
   {
	   window.setColor(Color.BLACK);
      window.drawRect(battery_Shell_xPos,battery_Shell_yPos, battery_Shell_Width, battery_Shell_Height);
      if (battery_Level>=66)
      {
         window.setColor(Color.WHITE);
         window.fillRect(battery_Shell_xPos+1,battery_Shell_yPos+1,battery_Level_Width,battery_Shell_Height-1);
      } else if (battery_Level<66)
      {
         if (battery_Level>=33)
         {
            window.setColor(Color.WHITE);
            window.fillRect(battery_Shell_xPos+1,battery_Shell_yPos+1,battery_Level_Width,battery_Shell_Height-1);
         } else {
            window.setColor(Color.WHITE);
            window.fillRect(battery_Shell_xPos+1,battery_Shell_yPos+1,battery_Level_Width,battery_Shell_Height-1);
         }
      }
      window.setColor(Color.BLACK);
      window.fillRect(battery_Shell_Cap_xPos,battery_Shell_Cap_yPos,battery_Shell_Cap_Width,battery_Shell_Cap_Height);
      window.setColor(Color.WHITE);
      window.setFont(new Font("TAHOMA",Font.BOLD,12));
      window.drawString(battery_Level+"%",30, 35);
      battery_Level=level;
      draw(window);
	}
}
