package bgu.dl.features.test;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;
import java.util.TreeMap;

public class hello 
{
	public static void main(String[] args) throws NumberFormatException, IOException {

		BufferedReader br = new BufferedReader(new FileReader("/home/shashank/Documents/Copy-IITM/Planning-Domain-IPC2/2000-Tests/Blocks/Generator/blocksworld-generator/bwstates/sss"));
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
 		
	}
}
