package robotik7;


import java.util.Vector;

public class Main {

	public static void main(String[] args) {
		
		Double tau = 1.5;
		Double T = 6.0;
		Double deltaT = 0.1;

		RhinoXR3Kinematic r = new RhinoXR3Kinematic();
		
		Vector<Double> startPunkt = new Vector<Double>();
		startPunkt.add(1.1786);
		startPunkt.add(0.0);
		startPunkt.add(0.3207);
		startPunkt.add(0.0);
		startPunkt.add(0.0);
		startPunkt.add(-0.60653);
		
		Vector<Double> endPunkt1 = new Vector<Double>();
		endPunkt1.add(0.0);
		endPunkt1.add(1.1786);
		endPunkt1.add(0.3207);
		endPunkt1.add(0.0);
		endPunkt1.add(0.0);
		endPunkt1.add(-0.60653);
		
		Vector<Double> endPunkt2 = new Vector<Double>();
		endPunkt2.add(0.0);
		endPunkt2.add(0.3969);
		endPunkt2.add(1.439);
		endPunkt2.add(0.0);
		endPunkt2.add(0.60653);
		endPunkt2.add(0.0);
		
		Vector<Double> wNeu = new Vector<Double>();
		Double speed;
		
		for(Double t = 0.0; t < T; t=t+deltaT){
			wNeu.clear();
			
			//w(t) = (1-s(t))w0+s(t)w1
			speed = r.speed(t, T, tau);
//			speed = t/T;
			wNeu.add((1-speed)*startPunkt.get(0)+speed*endPunkt1.get(0));
			wNeu.add((1-speed)*startPunkt.get(1)+speed*endPunkt1.get(1));
			wNeu.add((1-speed)*startPunkt.get(2)+speed*endPunkt1.get(2));
			wNeu.add((1-speed)*startPunkt.get(3)+speed*endPunkt1.get(3));
			wNeu.add((1-speed)*startPunkt.get(4)+speed*endPunkt1.get(4));
			wNeu.add((1-speed)*startPunkt.get(5)+speed*endPunkt1.get(5));
			System.out.println("t: "+t);
			System.out.println("w als q:\n"+wNeu);
		}
		
		
		
//		Vector<Double> w = new Vector<Double>();
//		w.add(0.0);
//		w.add(1.1786);
//		w.add(0.3207);
//		w.add(0.0);
//		w.add(0.0);
//		w.add(-0.60653);
//		
//		
//		
//		System.out.println("w nativ:\n"+w);
//		
//		w=r.getGelenkvariablenVektor(w);
//		
//		System.out.println("w als q:\n"+w);
//		
//		w=r.getToolConfigVektor(w);
//		
//		System.out.println("w zurück in w:\n"+w);
//		
//		w = new Vector<Double>();
//		w.add(1.1786);
//		w.add(0.0);
//		w.add(0.3207);
//		w.add(0.0);
//		w.add(0.0);
//		w.add(-0.60653);
//		
//		
//		
//		System.out.println("\n\nw nativ:\n"+w);
//		
//		w=r.getGelenkvariablenVektor(w);
//		
//		System.out.println("w als q:\n"+w);
//		
//		w=r.getToolConfigVektor(w);
//		
//		System.out.println("w zurück in w:\n"+w);
//		
//		w = new Vector<Double>();
//		w.add(0.0);
//		w.add(0.3969);
//		w.add(1.439);
//		w.add(0.0);
//		w.add(0.60653);
//		w.add(0.0);
//		
//		
//		
//		System.out.println("\n\nw nativ:\n"+w);
//		
//		w=r.getGelenkvariablenVektor(w);
//		
//		System.out.println("w als q:\n"+w);
//		
//		w=r.getToolConfigVektor(w);
//		
//		System.out.println("w zurück in w:\n"+w);
		
		

	}

}
