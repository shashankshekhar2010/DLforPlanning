package bgu.dl.features.test;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintStream;

import javax.swing.JOptionPane;

public class test1
{
	/*public static void main(String[] args) throws PyException
    {   
        try
        {
            PythonInterpreter.initialize(System.getProperties(), System.getProperties(), new String[0]);
            PythonInterpreter interp = new PythonInterpreter();
            interp.execfile("/home/Dropbox/src/features/learning/ss-py.py,1");
        }
        catch(Exception e)
        {
            System.out.println(e.toString());
            e.printStackTrace();
        }
    }*/
	public static void main(String a[]) throws IOException
	{
		/*	try {

			String prg = "import sys\nprint int(sys.argv[1]) + int(sys.argv[2])\n";
			BufferedWriter out = new BufferedWriter(new FileWriter("test1.py"));
			out.write(prg);
			out.close();
			int number1 = 10;
			int number2 = 32;
			// Process p = Runtime.getRuntime().exec("python test1.py "+number1+" "+number2);
			String[] command = {"python", "ss-py.py"};
			Process p = Runtime.getRuntime().exec("python ss-py.py");
			ProcessBuilder probuilder = new ProcessBuilder( command );
			Process p1 = probuilder.start();
			// Process p1 = Runtime.getRuntime().exec("python ss-py.py"); 

			BufferedReader reader = null;
			try {
				reader = new BufferedReader(new InputStreamReader(p1.getInputStream()));
				String line;
				while ((line = reader.readLine()) != null) {
					if (line.contains(searchFor)) {
						System.out.println(line);
					}
				}
			} finally {
				if (reader != null) {
					reader.close();
				}
			}
			// Writing each line to a file
			// System.setOut(new PrintStream(new BufferedOutputStream(new FileOutputStream("/home/Dropbox/eclipse/features/learning/ss")), true));
			BufferedReader in = new BufferedReader(new InputStreamReader(p.getInputStream()));
			String line;
			while ((line = in.readLine()) != null) {
				System.out.println(line);
			}
		}
		catch(Exception e){}

		 */
		try{
			String[] str = {"/home/Documents/Research-Edited/Fast-Downward/fast-downward.py",
					"/home/Dropbox/IPC-2/Blocks/Untyped/domain.pddl",
					"/home/Dropbox/IPC-2/Blocks/Untyped/probBLOCKS-4-0.pddl",
					"--search \"lazy_greedy(ff(), preferred=ff())\""};			
			Process p1 = Runtime.getRuntime().exec(str);
			/*BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(p1.getOutputStream()));
			writer.write("shekhar22!");
			writer.newLine();
			writer.close();*/
			//Process p = Runtime.getRuntime().exec(str, null, null);
			// ProcessBuilder probuilder = new ProcessBuilder( command );
			//Process process = probuilder.start(); 
			BufferedReader in = new BufferedReader(new InputStreamReader(p1.getInputStream()));
			String line;
			while ((line = in.readLine()) != null) {
				System.out.println(line);		}
		}catch(Exception e){

		}

	}
}
