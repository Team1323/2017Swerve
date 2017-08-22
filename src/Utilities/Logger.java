package Utilities;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Logger {
	public static void log(String mark) {

        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/log.txt", true))) {
        	
            writer.print(mark);
            writer.println();
            
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
