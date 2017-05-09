package bgu.dl.features.test;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class csv {

	/**
	 * @param args
	 * @throws FileNotFoundException 
	 */
	public static void main(String[] args) throws FileNotFoundException {
        Scanner scanner = new Scanner(new File("/home/shashank/Documents/Experiments-DL-NGrams/temp.csv"));
        scanner.useDelimiter(",");
        while (scanner.hasNext()) 
        {
            System.out.print(scanner.next() + "|");
        }
         
        //Do not forget to close the scanner  
        scanner.close();
	}
}