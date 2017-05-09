
package bgu.dl.features.collections;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import org.paukov.combinatorics.Factory;
import org.paukov.combinatorics.Generator;
import org.paukov.combinatorics.ICombinatoricsVector;

import pddl4j.PDDLObject;
import pddl4j.exp.AtomicFormula;
import pddl4j.exp.Exp;
import pddl4j.exp.term.Constant;
import bgu.dl.features.learning.MainClassForRandProblem;
import bgu.dl.features.learning.PossibleGroundedLiterals;
import bgu.dl.features.learning.ProblemDetails;

/**
 * @author Shashank Shekhar
 * BGU of the Negev
 * */
public class PropContextDataSet {
	ProblemDetails details;
	PDDLObject pddlObject;
	ArrayList<ArrayList<ArrayList>> listOfParentChild;
	ArrayList<PossibleGroundedActions> groundedActions; 	

	public PropContextDataSet() {
		this.details = null;
		this.pddlObject = null;
		this.groundedActions = new ArrayList<PossibleGroundedActions>();
	}

	public void dataSet(ProblemDetails details, PDDLObject object) {
		this.details = details;
		this.pddlObject = object;		
	}

	/**
	 *  Will be called from the main file!
	 * */
	public void callForDatasetGeneration(PlanDetails planDetails, Writer writer, Writer writer_ngram_context, MainClassForRandProblem problem) {
		PossibleGroundedLiterals possibleGroundedLiterals = new PossibleGroundedLiterals(pddlObject);		
		ArrayList<AtomicFormula> listOfPossiblePropositions = new ArrayList<>();
		listOfPossiblePropositions = possibleGroundedLiterals.allPossibleLiteralsMayOccur();
		this.details.generateGroundedActions();
		this.groundedActions = this.details.getgActions();

		try 
		{		
			useBootstrapping(planDetails, writer, writer_ngram_context, listOfPossiblePropositions, problem);
		} 
		catch (Exception e) {			
			System.out.println("Error in the main dataset preparation function call..."); 
			e.printStackTrace();
		}		
	}

	@SuppressWarnings({ "unchecked", "static-access" })
	void useBootstrapping(PlanDetails planDetails, Writer writer, Writer writer_ngram_context, 
			ArrayList<AtomicFormula> listOfPossiblePropositions_1, MainClassForRandProblem problem) {
		ArrayList<String> listOfPossiblePropositions = new ArrayList<>();
		ArrayList<String> header_init = new ArrayList<String>();
		ArrayList<String> header_goal = new ArrayList<String>();
		for (int k = 0; k < listOfPossiblePropositions_1.size(); k++) {
			header_init.add(listOfPossiblePropositions_1.get(k)+"-I");
			header_init.add("(not"+ listOfPossiblePropositions_1.get(k) + ")" +"-I");
			header_goal.add(listOfPossiblePropositions_1.get(k) +"-G");
			header_goal.add("(not"+ listOfPossiblePropositions_1.get(k) + ")" +"-G");
		}
		listOfPossiblePropositions.addAll(header_init);
		listOfPossiblePropositions.addAll(header_goal);	

		/**
		 * Keep in mind that, training data points will be computed using each child state.
		 * Call to the Fast Downward by passing the child state, and the goal state. **/
		// PossibleGroundedLiterals possibleGroundedLiterals = new PossibleGroundedLiterals(pddlObject);
		int target = planDetails.getPlanLength();
		ArrayList<String> plan = planDetails.getGeneratedRealPlan();

		// Get the real plan
		int count = 0;
		ArrayList<PossibleGroundedActions> realActions = new ArrayList<PossibleGroundedActions>();
		for (int i = 0; i < plan.size(); i++) {
			String actString = plan.get(i);  
			Iterator<PossibleGroundedActions> itr = groundedActions.iterator();
			while(itr.hasNext()) {
				PossibleGroundedActions ga = itr.next();
				boolean flag = ga.isThatGroundedAction(actString);
				if(flag) {
					realActions.add(ga);
					break;					
				}
			}
		}

		@SuppressWarnings("unchecked")
		ArrayList<AtomicFormula> theRealGoal = this.details.getGoalState();
		ArrayList<AtomicFormula> theRealGoalPrime = this.details.getInitialState();		
		for (int i = 0; i < realActions.size(); i++) {
			theRealGoalPrime = applyAction(realActions.get(i), theRealGoalPrime);
		}

		/** Hash map for the index. */
		HashMap<String,Integer> localIndexForEachLiteral = new HashMap<String, Integer>();  
		for(int i=0; i<listOfPossiblePropositions.size(); i++) {
			localIndexForEachLiteral.put(listOfPossiblePropositions.get(i), i+1);
		}	

		/**
		 * Generate different combinations of those extra achievements, as of now consider this combination generator. */
		ArrayList<Integer> listWithDontCare = new ArrayList<Integer>();
		for (int i = 0; i < listOfPossiblePropositions.size(); i++) {			
			boolean flag = true;
			for (int j = 0; j < theRealGoal.size(); j++) {
				String str = theRealGoal.get(j).toString()+"-G";
				if(listOfPossiblePropositions.get(i).equals(str)) {
					listWithDontCare.add(1); flag = false; break;
				}
			}
			if(flag)
				listWithDontCare.add(0);
		}

		ArrayList<ArrayList<Integer>> allPossibleListWithDontCare = new ArrayList<ArrayList<Integer>>();
		allPossibleListWithDontCare.add(listWithDontCare);		// just adds the original goal.

		/** goalPrimeMinusGoal = set operation of {theRealGoalPrime \ theRealGoal} */
		ArrayList<AtomicFormula> goalPrimeMinusGoal = new ArrayList<AtomicFormula>();
		for (int i = 0; i < theRealGoalPrime.size(); i++) {
			boolean flag = false;
			for (int j = 0; j < theRealGoal.size(); j++) {
				if(theRealGoalPrime.get(i).equals(theRealGoal.get(j))){
					flag = true; break;
				}				
			}
			if (!flag) {
				goalPrimeMinusGoal.add(theRealGoalPrime.get(i));
			}
		}

		/** Generated all possible 0s and 1s from the extra that we achieve. */
		ArrayList<ArrayList<AtomicFormula>> generateAllPossibleCombiOfExtraGoalAchieved = new ArrayList<ArrayList<AtomicFormula>>(); 
		Set<Set<AtomicFormula>>	af = generateAllPossibleCombiOfExtraGoalAchieved(goalPrimeMinusGoal);
		for (Set<AtomicFormula> s : af) {
			if(s.size() !=0) {
				ArrayList<AtomicFormula> f = new ArrayList<>(s);
				generateAllPossibleCombiOfExtraGoalAchieved.add(f);
			}
		}

		for (int i = 0; i < generateAllPossibleCombiOfExtraGoalAchieved.size(); i++) 
		{	
			ArrayList<AtomicFormula> oneListWithExtraGoalLiterals = generateAllPossibleCombiOfExtraGoalAchieved.get(i);
			ArrayList<Integer> oneListCompl_IG_WithExtraGoalLiterals = new ArrayList<Integer>();
			oneListCompl_IG_WithExtraGoalLiterals.addAll(listWithDontCare);			
			for (int j2 = 0; j2 < oneListWithExtraGoalLiterals.size(); j2++) {
				String string = oneListWithExtraGoalLiterals.get(j2).toString()+"-G";
				Integer ind = localIndexForEachLiteral.get(string);
				oneListCompl_IG_WithExtraGoalLiterals.set(ind-1, 1);
			}			
			allPossibleListWithDontCare.add(oneListCompl_IG_WithExtraGoalLiterals);
		}	

		/** Code for bootstrapping! */
		ArrayList<AtomicFormula> forwardState = new ArrayList<AtomicFormula>();	
		forwardState.addAll(this.details.getInitialState());
		// System.out.println("Actual target: "+ target +" Size(Powerset): "+ allPossibleListWithDontCare.size());
		
		while(target >=0 ) {			
			// This captures the initial state (listOfIntegersCorrespondingToLiterals).
			ArrayList<Integer> listOfIntegersCorrespondingToLiterals = generateDataset(forwardState, listOfPossiblePropositions, localIndexForEachLiteral);	
			
			for (int i = 0; i < allPossibleListWithDontCare.size(); i++) {				
				ArrayList<Integer> list = new ArrayList<Integer>();	
				for (int j = 0; j < listOfPossiblePropositions.size(); j++) 
					list.add(0);
				int size = listOfPossiblePropositions.size();
				for (int j = 0; j < size/2; j++) {
					list.set(j, listOfIntegersCorrespondingToLiterals.get(j));
					list.set(size/2+j, allPossibleListWithDontCare.get(i).get(size/2+j));
				}
				
				// Call for the N-Gram context generation in a given (I, G) pair.
				// callForTheContextOfTheNGramLiteralsInTheCurrentInitGoalStates(list, writer_ngram_context);
				
				list.add(target);
				String data = "";			
				for (int r = 0; r < list.size(); r++) {
					data = data + list.get(r) + "\t";
				}	
				data = data + "\n";
				try {
					writer.append(data); 
				} catch (IOException e) {
					System.out.println("Error in the useBootstrapping function call");
					e.printStackTrace();
				}
				problem.numberOfSamplesGenerated++;
			}

			// Move to the next state using an action from the generated plan. 
			if(target != 0) {
				String actString1 = plan.get(count); 
				Iterator<PossibleGroundedActions> itr1 = groundedActions.iterator();
				while(itr1.hasNext()) {
					PossibleGroundedActions ga = itr1.next();
					boolean flag = ga.isThatGroundedAction(actString1);
					if(flag) {
						forwardState = applyAction(ga, forwardState);
						count++; break;					
					}
				}
			}				
			target--; 
		}
	}

	/**
	 * Maintaining context in each state.
	 * */
	void callForTheContextOfTheUnigramLiteralsInTheCurrentInitGoalStates(ArrayList<Integer>listOfIntegersCorrespondingToLiterals, Writer writer_context)
	{
		String data = "";	
		data = listOfIntegersCorrespondingToLiterals.toString();
		for (int r = 0; r < listOfIntegersCorrespondingToLiterals.size(); r++) {
			data = data + listOfIntegersCorrespondingToLiterals.get(r) + "\t";
		}	
		data = data + "\n";
		try {
			writer_context.append(data); 
		} catch (IOException e) {			
			e.printStackTrace();
			System.out.println("Error while writing the contexts");
		}		
	}

	/**
	 * Maintaining context in each state.
	 * @throws IOException 
	 * */
	void callForTheContextOfTheNGramLiteralsInTheCurrentInitGoalStates(ArrayList<Integer> listOfIntegersCorrespondingToLiterals, Writer writer_ngram_context) 
	{
		File file = new File("/home/shashank/Documents/Experiments-DL-NGrams/temp.txt");
		String data = "";
		try {
			Writer writer = new BufferedWriter(new FileWriter(file));
			for (int r = 0; r < listOfIntegersCorrespondingToLiterals.size(); r++) {
				for (int s = r+1; s < listOfIntegersCorrespondingToLiterals.size(); s++) {
					if(listOfIntegersCorrespondingToLiterals.get(r) == 1 && listOfIntegersCorrespondingToLiterals.get(s) == 1)
						writer.append("1" + "\t");					
					else 
						writer.append("0" + "\t");				
				}			
			}
			writer.append("\n");
			writer.close();
		} catch (IOException e1) {
			e1.printStackTrace();
		}

		try {
			data = new String(Files.readAllBytes(Paths.get("/home/shashank/Documents/Experiments-DL-NGrams/temp.txt")));
			writer_ngram_context.append(data);
			
		} catch (IOException e) {			
			e.printStackTrace();
			System.out.println("Error while writing the contexts");
		}		
	}

	/**
	 * Function call to the fast-downward (FD) planner | keep in mind that the call gets killed after a certain time (say after 30 minutes). 
	 * @param initialState
	 * @param goalState
	 * @return returns the target value, basically, the plan length found by the FD planner.
	 * */
	@SuppressWarnings({ "rawtypes", "unused" })
	private PlanDetails targetByFastDownward(ArrayList initialState, PossibleGroundedLiterals possibleGroundedLiterals) {
		int target = 1000000; // default path length
		ArrayList<String> plan = new ArrayList<String>();
		PlanDetails details =  new PlanDetails();		
		// A call to the fast-downward through python script.
		try {
			String[] command = {
					"/home/shashank/Documents/Copy-IITM/Research-Edited/Fast-Downward/fast-downward.py",
					"/home/shashank/Dropbox/Bgu-Files/bgu.dl.heuristic/eclipse/bgu.learning/src/bgu/dl/features/learning/domain.pddl",
					"/home/shashank/Dropbox/Bgu-Files/bgu.dl.heuristic/eclipse/bgu.learning/src/bgu/dl/features/learning/problem.pddl",
					"--heuristic",
					"h=ff()",
					"--search",
					"lazy_greedy(h, preferred=h)"
			};

			Process pro = Runtime.getRuntime().exec(command);
			BufferedReader in = new BufferedReader(new InputStreamReader(pro.getInputStream()));
			String line = null;
			String[] planDetails = null;
			boolean firstLine = false;
			boolean secondLine = false;
			int count = 0;
			while ((line = in.readLine()) != null) {
				// Conditions for getting the real plans
				if(firstLine) 
					count++;
				if (line.contains("Actual search")) 
					firstLine = true;
				if (line.contains("Plan length"))  
					secondLine = true;

				if(firstLine && count >= 1 && !secondLine) {					
					String[] currAction = line.split(" ");
					String str = "(";
					for (int i = 0; i < currAction.length; i++) {
						if (i < currAction.length-1) { 
							if(i < currAction.length-2) 
								str = str + currAction[i] + " ";
							else
								str = str + currAction[i];
						}
					}
					str = str + ")";		
					plan.add(str.toString());
				}
				// Conditions for getting the real targets predicted by FD.
				if (line.contains("Plan length")) {
					planDetails = line.split(" ");
				}
			}	
			target = Integer.parseInt(planDetails[2]);
			details.setPlanLength(target);
			details.setGeneratedRealPlan(plan);
			// The real plan extraction is ridiculously done, really need an update on that!
		} catch (Exception e) {
			System.err.println("Error in writing the planner output in file !!");
		}
		return details; 
	}

	/**
	 * Will write a new initial state 
	 * @param initialState
	 * @param goalState
	 * @param listOfConstants */
	private void generateProblemFile(ArrayList initialState) 
	{ 
		List<String> lines = new ArrayList<String>();
		String line = null;
		try {
			File f1 = new File("/home/shashank/Dropbox/Bgu-Files/bgu.dl.heuristic/eclipse/bgu.learning/src/bgu/dl/features/learning/problem.pddl");
			FileReader fr = new FileReader(f1);
			BufferedReader br = new BufferedReader(fr);
			String str ="(:init ";
			for (int i = 0; i < initialState.size(); i++) {
				str = str + initialState.get(i).toString();
			}
			str = str + ")";
			while ((line = br.readLine()) != null) {
				if (line.contains(":init") || line.contains(":INIT") )
				{
					line = line.replace(line, str);
				}
				lines.add(line);
			}
			fr.close();
			br.close();

			FileWriter fw = new FileWriter(f1);
			BufferedWriter out = new BufferedWriter(fw);
			for(String s : lines)
			{
				out.write(s);
				out.write("\n");
			}
			out.flush();
			out.close();
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}

	// Calling to feed (0 or 1) corresponding to each entry in the list of proposition
	private ArrayList<Integer> generateDataset(ArrayList<AtomicFormula> givenState, ArrayList<String> listOfPossiblePropositions, HashMap<String, Integer> indexMap) {	
		// No closed world assumptions - basically.
		ArrayList<Integer> listOfInt = new ArrayList<Integer>();
		for (int i = 0; i < listOfPossiblePropositions.size()/2; i++) {
			listOfInt.add(0); // by default all 0s			
		}
		for (int i = 0; i < givenState.size(); i++) {
			String string = givenState.get(i).toString();
			string = string+"-I";
			Integer ind = indexMap.get(string);
			listOfInt.set(ind-1, 1);
		}
		for (int i = 0; i < listOfPossiblePropositions.size()/2; i++) {
			String af = listOfPossiblePropositions.get(i);
			if(af.contains("not") && af.contains("-I")) {
				if(listOfInt.get(i-1) == 0) {
					listOfInt.set(i, 1);	
				}
				continue;
			}
		}		
		return listOfInt;
	}

	/**
	 * Check whether state g is a subset of current state currentState
	 * @param goalState
	 * @param currentState
	 * @return status on whether g is subset is current. */
	private boolean isSubsetOf(ArrayList<AtomicFormula> goalState, ArrayList<AtomicFormula> currentState) {
		return(currentState.containsAll(goalState));		
	}

	/**
	 * Applies a grounded action on the given state
	 * @param groundedAction
	 * @param childNode
	 * @return a successor state */
	private ArrayList<AtomicFormula> applyAction(PossibleGroundedActions groundedAction, ArrayList<AtomicFormula> childNode) {
		ArrayList<AtomicFormula> successorState = new ArrayList<AtomicFormula>();
		successorState.addAll(childNode);
		ArrayList<Exp> removeNeg = new ArrayList<Exp>();
		// groundedAction.printGroundedAction();

		/** Formula: S_{new} = {S_{current} - effect^{-}(a)} U {effect^{+}(a)} */
		Iterator<AtomicFormula> itrNew = successorState.iterator();
		while(itrNew.hasNext()) {
			Exp exp = itrNew.next();
			if(groundedAction.getNegEff().contains(exp))
				removeNeg.add(exp);
		}

		/** Set Operations */
		successorState.removeAll(removeNeg);
		successorState.addAll(groundedAction.getPosEff());		
		return successorState;
	}

	/**
	 * Returns all possible combinations of extra goal predicates achieved during the plan execution.
	 * @param apartFromTheGoals
	 * @return generateAllPossibleCombiOfExtraGoalAchieved **/
	public ArrayList<ArrayList<AtomicFormula>> generateAllPossibleCombiOfExtraGoalAchieved_old(ArrayList<AtomicFormula> apartFromTheGoals) {
		ArrayList<ArrayList<AtomicFormula>> generateAllPossibleCombiOfExtraGoalAchieved = new ArrayList<ArrayList<AtomicFormula>>();
		ArrayList<AtomicFormula> eachCombiOfExtraGoals = new ArrayList<AtomicFormula>();
		for (int i = 0; i < apartFromTheGoals.size(); i++) {
			int count = i+1;
			ICombinatoricsVector<AtomicFormula> combinatoricsVector = Factory.createVector(apartFromTheGoals);
			Generator<AtomicFormula> generator = Factory.createSimpleCombinationGenerator(combinatoricsVector, count);

			/** Read all possible combinations */
			for (ICombinatoricsVector<AtomicFormula> combination : generator) 
			{
				java.util.List<AtomicFormula> l = combination.getVector();
				ICombinatoricsVector<AtomicFormula> temp = Factory.createVector(l);
				Generator<AtomicFormula> genPerm = Factory.createPermutationGenerator(temp);
				for (ICombinatoricsVector<AtomicFormula> perm : genPerm) {
					java.util.List<AtomicFormula> p = perm.getVector();
					eachCombiOfExtraGoals = new ArrayList<AtomicFormula>();
					Iterator<AtomicFormula> itr = p.iterator();
					while(itr.hasNext()) {
						eachCombiOfExtraGoals.add(itr.next());				
					}
					generateAllPossibleCombiOfExtraGoalAchieved.add(eachCombiOfExtraGoals);
				}
			}
		}
		return generateAllPossibleCombiOfExtraGoalAchieved;
	}

	// Re-written
	public Set<Set<AtomicFormula>> generateAllPossibleCombiOfExtraGoalAchieved(ArrayList<AtomicFormula> apartFromTheGoals) 
	{
		Set<AtomicFormula> set = new HashSet<AtomicFormula>();
		set.addAll(apartFromTheGoals);
		Set<Set<AtomicFormula>> sets = new HashSet<Set<AtomicFormula>>();
		if (apartFromTheGoals.isEmpty()) {
			sets.add(new HashSet<AtomicFormula>());
			return sets;
		}
		List<AtomicFormula> list = new ArrayList<AtomicFormula>(apartFromTheGoals);
		AtomicFormula head = list.get(0);
		Set<AtomicFormula> rest = new HashSet<AtomicFormula>(list.subList(1, list.size()));
		ArrayList<AtomicFormula> af = new ArrayList<AtomicFormula>(rest);
		for (Set<AtomicFormula> set1 : generateAllPossibleCombiOfExtraGoalAchieved(af)) {
			Set<AtomicFormula> newSet = new HashSet<AtomicFormula>();
			newSet.add(head);
			newSet.addAll(set1);
			sets.add(newSet);
			sets.add(set1);
		}       
		return sets;
	}
}
