package robotik7;

import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.pow;

import java.util.Vector;

public class Main {

	public static void main(String[] args) {

		RhinoXR3Kinematic r = new RhinoXR3Kinematic();
		
//		Vector<Double> q = new Vector<Double>();
//		q.add(1.3);
//		q.add(1.7);
//		q.add(1.9);
//		q.add(2.3);
//		q.add(2.7);
//		
//		
//		System.out.println(pow(-E, (1.5))+"  pow\n");
//		System.out.println("q nativ:\n"+q);
//		
//		q=r.getToolConfigVektor(q);
//		
//		System.out.println("q als w:\n"+q);
//		
//		q=r.getGelenkvariablenVektor(q);
//		
//		System.out.println("q zurück in q:\n"+q);
//		
		Vector<Double> w = new Vector<Double>();
		w.add(0.0);
		w.add(1.1786);
		w.add(0.3207);
		w.add(0.0);
		w.add(0.0);
		w.add(-0.60653);
		
		
		
		System.out.println("w nativ:\n"+w);
		
		w=r.getGelenkvariablenVektor(w);
		
		System.out.println("w als q:\n"+w);
		
		w=r.getToolConfigVektor(w);
		
		System.out.println("w zurück in w:\n"+w);
		

	}

}
