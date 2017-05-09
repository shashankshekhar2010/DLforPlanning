package bgu.dl.features.test;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

public class hello 
{
	public static void main(String[] args) throws Exception {

		String path = "/home/shashank/Documents/Copy-IITM/Planning-Domain-IPC2/2000-Tests/Blocks/Generator/blocksworld-generator/bwstates/ssss";
		BufferedReader br = new BufferedReader(new FileReader(path));
		Map<Integer, String> items = new TreeMap<Integer, String>();

		String line = new String();
		ArrayList<ArrayList<String>> list1 = new ArrayList<ArrayList<String>>();
		ArrayList<String> list;
		while (null != (line = br.readLine()))
		{
			String[] line_parts = line.split(" ");
			list = new ArrayList<String>();
			for (int i = 0; i < line_parts.length; i++)
			{
				list.add((String)line_parts[i]);
			}
			// System.out.println(list);
			list1.add(list);
		}
		
		hello hello1 = new hello();
		hello1.stripDuplicatesFromFile(path);

	}

	public void stripDuplicatesFromFile(String filename) throws Exception { 
		BufferedReader reader = 
				new 
				BufferedReader(new FileReader(filename));
		Set<String> lines = new HashSet<String>(10000); // maybe should be bigger
		String line;
		while ((line = reader.readLine()) != null) {
			lines.add(line);
		}
		reader.close();
		BufferedWriter writer = new BufferedWriter(new FileWriter(filename));
		for (String unique : lines) {
			writer.write(unique);
			writer.newLine();
		}
		writer.close();
	}
}
