package robotik7;


import java.util.Vector;

public class Main {

	public static void main(String[] args) {

		RhinoXR3Kinematic r = new RhinoXR3Kinematic();
		
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
		
		w = new Vector<Double>();
		w.add(1.1786);
		w.add(0.0);
		w.add(0.3207);
		w.add(0.0);
		w.add(0.0);
		w.add(-0.60653);
		
		
		
		System.out.println("\n\nw nativ:\n"+w);
		
		w=r.getGelenkvariablenVektor(w);
		
		System.out.println("w als q:\n"+w);
		
		w=r.getToolConfigVektor(w);
		
		System.out.println("w zurück in w:\n"+w);
		
		w = new Vector<Double>();
		w.add(0.0);
		w.add(0.3969);
		w.add(1.439);
		w.add(0.0);
		w.add(0.60653);
		w.add(0.0);
		
		
		
		System.out.println("\n\nw nativ:\n"+w);
		
		w=r.getGelenkvariablenVektor(w);
		
		System.out.println("w als q:\n"+w);
		
		w=r.getToolConfigVektor(w);
		
		System.out.println("w zurück in w:\n"+w);
		
		

	}

}
