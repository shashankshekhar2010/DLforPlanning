package bgu.dl.features.learning;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.Writer;
import java.util.Properties;

import bgu.dl.features.collections.PlanDetails;
import bgu.dl.features.collections.PropContextDataSet;

import pddl4j.Domain;
import pddl4j.ErrorManager;
import pddl4j.ErrorManager.Message;
import pddl4j.PDDLObject;
import pddl4j.Parser;
import pddl4j.Problem;
import pddl4j.RequireKey;

/**
 * @author Shashank Shekhar
 * BGU of the Negev
 * @throws FileNotFoundException 
 * */
public class InstantiatedByRandomProblem {	
	public PDDLObject instantiationUsingARandomProblem(String domainPath, String problemPath) throws FileNotFoundException {		
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

		Parser parser = new Parser(properties);		

		// Domain domainRelaxed = parser.parse(new File(args[0]));			
		Domain domain = parser.parse(new File(domainPath));			
		Problem problem = parser.parse(new File(problemPath));

		// PDDLObject objRelaxed = parser.link(domainRelaxed, problem);
		PDDLObject obj = parser.link(domain, problem);
		// Gets the error manager of the pddl parser
		ErrorManager mgr = parser.getErrorManager();

		// If the parser produces errors we print it and stop
		if (mgr.contains(Message.ERROR)) {
			mgr.print(Message.ALL); 
			System.exit(0);
		}
		return obj;
	}
	void callForDatasetGeneration(PlanDetails planDetails, PDDLObject obj, Writer writer, Writer writer_ngram_context, MainClassForRandProblem problem)
	{		
		ProblemDetails details = new ProblemDetails(obj);
		PropContextDataSet set = new PropContextDataSet();
		try {
			set.dataSet(details, obj);
			set.callForDatasetGeneration(planDetails, writer, writer_ngram_context, problem);
		} catch (Exception e) {
			System.err.println("--------------- Error in the instantiationUsingARandomProblem() call -----------------"); 
		}			
	}
}
