package bgu.dl.features.learning;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Properties;

import bgu.dl.features.collections.DataSet_Type_1;
import bgu.dl.features.collections.DataSet_Type_2;

import pddl4j.Domain;
import pddl4j.ErrorManager;
import pddl4j.ErrorManager.Message;
import pddl4j.PDDLObject;
import pddl4j.Parser;
import pddl4j.Problem;
import pddl4j.RequireKey;

public class MainEntry {

	/**
	 * @author Shashank Shekhar
	 * BGU of the Negev
	 * @throws FileNotFoundException 
	 */
	public static void main(String[] args) throws FileNotFoundException {
		System.out.println("--------------");
		System.out.println("  Hello BGU   ");
		System.out.println("--------------");
		long begin = System.nanoTime();	
		Properties properties = new Properties();

		properties.put(RequireKey.STRIPS, true);
		properties.put(RequireKey.EQUALITY, true);
		properties.put(RequireKey.ADL, true);
		properties.put(RequireKey.CONDITIONAL_EFFECTS, true);
		properties.put(RequireKey.CONSTRAINTS, true);
		properties.put(RequireKey.CONTINUS_EFFECTS, true);
		properties.put(RequireKey.DERIVED_PREDICATES, true);
		properties.put(RequireKey.DISJUNCTIVE_PRECONDITIONS, true);
		properties.put(RequireKey.DURATION_INEQUALITIES, true);
		properties.put(RequireKey.DURATIVE_ACTIONS, true);
		properties.put(RequireKey.EXISTENTIAL_PRECONDITIONS, true);
		properties.put(RequireKey.FLUENTS, true);
		properties.put(RequireKey.NEGATIVE_PRECONDITIONS, true);
		properties.put(RequireKey.PREFERENCES, true);
		properties.put(RequireKey.QUANTIFIED_PRECONDITIONS, true);
		properties.put(RequireKey.QUANTIFIED_PRECONDITIONS, true);
		properties.put(RequireKey.TIMED_INITIAL_LITERALS, true);
		properties.put(RequireKey.UNIVERSAL_PRECONDITIONS, true);

		// Check for the arguments if the files are not proper
		if (args.length != 2) 
		{
			System.err.print("The number of arguments are " + args.length);
			System.err.println(", please check the requirements that you need to pass");
			System.exit(0);
		}

		// Creates an instance of the java pddl parser
		System.out.println("Parser is placed now!"); 
		Parser parser = new Parser(properties);		

		// Domain domainRelaxed = parser.parse(new File(args[0]));			
		Domain domain = parser.parse(new File(args[0]));			
		Problem problem = parser.parse(new File(args[1]));

		// PDDLObject objRelaxed = parser.link(domainRelaxed, problem);
		PDDLObject obj = parser.link(domain, problem);

		// Gets the error manager of the pddl parser
		ErrorManager mgr = parser.getErrorManager();

		// If the parser produces errors we print it and stop
		if (mgr.contains(Message.ERROR)) {
			mgr.print(Message.ALL); 
			System.exit(0);
		} 
		else { 
			mgr.print(Message.WARNING);
			System.out.println("\nParsing domain \"" + domain.getDomainName() + "\" done successfully ...");
			System.out.println("Parsing problem \"" + problem.getProblemName()+ "\" done successfully ...\n");

			// Populate the details-class with the provided information.
			ProblemDetails details = new ProblemDetails(obj);

			/** Calls for different training set preparations. */
			// DataSet_Type_1 set = new DataSet_Type_1();
			DataSet_Type_2 set =  new DataSet_Type_2();

			try {
				set.dataSet(details, obj);
				set.callForDatasetGeneration();
			} catch (Exception e) {
				System.err.println("--------------- Error in main call -----------------\n"); 
			}			
		}	
	}
}
