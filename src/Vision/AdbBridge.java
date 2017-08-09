package Vision;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * AdbBridge interfaces to an Android Debug Bridge (adb) binary, which is needed
 * to communicate to Android devices over USB.
 *
 * adb binary provided by https://github.com/Spectrum3847/RIOdroid
 */
public class AdbBridge {
    Path bin_location_;
    public final static Path DEFAULT_LOCATION = Paths.get("/usr/bin/adb");

    public AdbBridge() {
        Path adb_location;
        String env_val = System.getenv("FRC_ADB_LOCATION");
        if (env_val == null || "".equals(env_val)) {
            adb_location = DEFAULT_LOCATION;
        } else {
            adb_location = Paths.get(env_val);
        }
        bin_location_ = adb_location;
    }

    public AdbBridge(Path location) {
        bin_location_ = location;
    }

    private String runCommand(String args) {
        Runtime r = Runtime.getRuntime();
        String cmd = bin_location_.toString() + " " + args;
        StringBuffer output = new StringBuffer();

        try {
            Process p = r.exec(cmd);
            p.waitFor();
            BufferedReader reader = new BufferedReader(new InputStreamReader(p.getInputStream()));
            String line = "";
            while((line = reader.readLine()) != null){
            	output.append(line + "\n");
            }
        } catch (IOException e) {
            System.err.println("AdbBridge: Could not run command " + cmd);
            e.printStackTrace();
            return "";
        } catch (InterruptedException e) {
            System.err.println("AdbBridge: Could not run command " + cmd);
            e.printStackTrace();
            return "";
        }
        return output.toString();
    }

    public void start() {
        System.out.println("Starting adb");
        runCommand("start");
    }

    public void stop() {
        System.out.println("Stopping adb");
        runCommand("kill-server");
    }

    public void restartAdb() {
        System.out.println("Restarting adb");
        stop();
        start();
    }

    public void portForward(int local_port, int remote_port) {
        runCommand("forward tcp:" + local_port + " tcp:" + remote_port);
    }

    public void reversePortForward(int remote_port, int local_port) {
        runCommand("reverse tcp:" + remote_port + " tcp:" + local_port);
    }

    public void restartApp() {
        System.out.println("Restarting app");
        runCommand("shell am force-stop com.team254.cheezdroid \\; "
                + "am start com.team254.cheezdroid/com.team254.cheezdroid.VisionTrackerActivity");
    }
    public String getBatteryPercentage(){
    	String output = runCommand("shell dumpsys battery");
    	if(output.length() > 0){
	    	String[] lines = output.split(System.getProperty("line.separator"));
	    	output = lines[8].replaceAll("[^0-9]+", "");
	    	output += "%";
	    	return output;
    	}
    	return "Error";
    }
    public boolean phoneConnected(){
    	String output = runCommand("devices");
    	if(output.length() > 0){
    		output = output.replace("List of devices attached", "");
    		if(output.contains("device")){
    			return true;
    		}
    	}
    	return false;
    }
}
