package bgu.dl.features.learning;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Set;

import org.paukov.combinatorics.Factory;
import org.paukov.combinatorics.Generator;
import org.paukov.combinatorics.ICombinatoricsVector;

import pddl4j.PDDLObject;
import pddl4j.exp.AndExp;
import pddl4j.exp.AtomicFormula;
import pddl4j.exp.Exp;
import pddl4j.exp.Literal;
import pddl4j.exp.term.Constant;
import pddl4j.exp.term.Substitution;
import pddl4j.exp.term.Term;
import pddl4j.exp.term.Variable;

/**
 * @author Shashank Shekhar
 * BGU of the Negev
 */
public class PossibleGroundedLiterals 
{
	PDDLObject pddlObject = null;
	public PossibleGroundedLiterals(PDDLObject pddlObject){
		this.pddlObject = pddlObject;
	}

	/**
	 * Given a search space, it will return you all the possible propositions that may occur
	 * @return list of literals
	 */
	public ArrayList<AtomicFormula> allPossibleLiteralsMayOccur() 
	{	
		ArrayList<AtomicFormula> allPossibleLiterals = new ArrayList<AtomicFormula>();
		ArrayList<AtomicFormula> ungroundedLiterals = listOfUngroundedLiterals();
		Iterator<AtomicFormula> iterator = ungroundedLiterals.iterator();
		while (iterator.hasNext()) {
			AtomicFormula atomicFormula = (AtomicFormula) iterator.next();
			allPossibleLiterals.addAll(allPossibleFormOfaLiteral(atomicFormula));		
		}		
		return allPossibleLiterals;
	}

	// Returns all possible combinations for a literal using all the constants
	private ArrayList<AtomicFormula> allPossibleFormOfaLiteral(AtomicFormula literal) {
		ArrayList<AtomicFormula> allPossibleFormOfaLiteral = new ArrayList<AtomicFormula>();
		ArrayList<Constant> constants = listOfConstants();
		Set<Variable> freeVar = literal.getFreeVariables();
		int noOfFreeVar = freeVar.size();
		if (noOfFreeVar == 0)
		{
			allPossibleFormOfaLiteral.add(literal);
		}
		Literal lit = (Literal) literal;
		ArrayList<ArrayList<Constant>> allPossibleCombi = generateAllCombinations(constants, noOfFreeVar);
		Iterator<ArrayList<Constant>> iterator = allPossibleCombi.iterator();
		// iterator will have something like [A, B]
		while (iterator.hasNext()) {
			ArrayList<Constant> listCons = iterator.next();
			Iterator freeVariableIterator = freeVar.iterator();
			Iterator listConstants = listCons.iterator();
			Substitution theta = new Substitution();
			while (freeVariableIterator.hasNext() && listConstants.hasNext()) {
				theta.bind((Variable)freeVariableIterator.next(), (Term)listConstants.next());				
			}
			AtomicFormula af = literal.apply(theta);
			allPossibleFormOfaLiteral.add((AtomicFormula)af);
		}		
		return allPossibleFormOfaLiteral;
	}	
	
	/**
	 * This function generates combinations of constants, depending on the number selected (nCr).
	 * @param constants
	 * @param noOfFreeVar
	 * @return All combinations of constants*/
	private ArrayList<ArrayList<Constant>> generateAllCombinations(ArrayList constants, int noOfFreeVar)  
	{
		ArrayList<ArrayList<Constant>> combi = new ArrayList<ArrayList<Constant>>();
		// Create the initial vector
		String freeVar[] = new String[constants.size()];
		for(int i=0,j=0;j<constants.size();i++,j++) {
			freeVar[i]=constants.get(j).toString();
		}
		ICombinatoricsVector<String> initialVector = Factory.createVector(freeVar);
		// Create a simple combination generator to generate noFreeVar-combinations of the initial vector
		Generator<String> gen = Factory.createSimpleCombinationGenerator(initialVector, noOfFreeVar);

		// Read all possible combinations
		for (ICombinatoricsVector<String> combination : gen) {
			java.util.List<String> l = combination.getVector();
			ICombinatoricsVector<String> temp = Factory.createVector(l);
			Generator<String> genPerm = Factory.createPermutationGenerator(temp);
			for (ICombinatoricsVector<String> perm : genPerm) {
				java.util.List<String> p = perm.getVector();
				ArrayList<Constant> c = new ArrayList<Constant>();
				Iterator<String> itr = p.iterator();
				while(itr.hasNext()) {
					c.add(new Constant(itr.next()));	 			
				}
				combi.add(c);
			}
		}	
		return combi;
	}

	// returns all constants
	public ArrayList<Constant> listOfConstants() {
		ArrayList<Constant> listOfCons = new ArrayList<Constant>();
		Iterator<Constant> iterator = pddlObject.constantsIterator();
		while(iterator.hasNext())
			listOfCons.add(iterator.next());
		return listOfCons;
	}

	// returns all predicates.
	private ArrayList<AtomicFormula> listOfUngroundedLiterals() {
		ArrayList<AtomicFormula> listOfUngroundedLiterals = new ArrayList<>();
		Iterator<AtomicFormula> itr = pddlObject.predicatesIterator();
		while(itr.hasNext()) 
			listOfUngroundedLiterals.add((AtomicFormula)itr.next());		
		return listOfUngroundedLiterals;
	}

}
