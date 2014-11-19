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
		Vector<Double> wLast = (Vector<Double>) startPunkt.clone();
		Vector<Double> sollSpeedTool = new Vector<Double>();
		Vector<Double> sollSpeedGelenk = new Vector<Double>();
		Vector<Double> gelenkNeu = new Vector<Double>();
		Vector<Double> qt = new Vector<Double>();
		Vector<Double> qtTOwt = new Vector<Double>();
		Double speed;
		
		for(Double t = 0.0; t < T; t=t+deltaT){
			//Nächsten Tool Config vector berechnen
			//w(t) = (1-s(t))w0+s(t)w1
			wNeu.clear();
			//Geschwindigkeitsverteilfunktion
			speed = r.speed(t, T, tau);
			wNeu.add((1-speed)*startPunkt.get(0)+speed*endPunkt1.get(0));
			wNeu.add((1-speed)*startPunkt.get(1)+speed*endPunkt1.get(1));
			wNeu.add((1-speed)*startPunkt.get(2)+speed*endPunkt1.get(2));
			wNeu.add((1-speed)*startPunkt.get(3)+speed*endPunkt1.get(3));
			wNeu.add((1-speed)*startPunkt.get(4)+speed*endPunkt1.get(4));
			wNeu.add((1-speed)*startPunkt.get(5)+speed*endPunkt1.get(5));
			System.out.println("t: "+t);
			System.out.println("neues w:\n"+wNeu);
			
			//Sollgeschwindigkeit im Tool Configraum berechnen
			sollSpeedTool.clear();
			sollSpeedTool.add((wNeu.get(0)-wLast.get(0))/deltaT);
			sollSpeedTool.add((wNeu.get(1)-wLast.get(1))/deltaT);
			sollSpeedTool.add((wNeu.get(2)-wLast.get(2))/deltaT);
			sollSpeedTool.add((wNeu.get(3)-wLast.get(3))/deltaT);
			sollSpeedTool.add((wNeu.get(4)-wLast.get(4))/deltaT);
			sollSpeedTool.add((wNeu.get(5)-wLast.get(5))/deltaT);
			System.out.println("Sollgeschwindigkeit Tool:\n"+sollSpeedTool);
			
			//Sollgeschwindigkeit im Gelenkraum
			sollSpeedGelenk.clear();
			qt = r.getGelenkvariablenVektor(wLast);
			sollSpeedGelenk = r.getGeschwindigkeitGelenkvariabel(wLast, sollSpeedTool, qt);
			System.out.println("Sollgeschwindigkeit Gelenk:\n"+sollSpeedGelenk);
			
			//Neue Position im Gelenkraum simulieren
			gelenkNeu.clear();
			gelenkNeu.add(qt.get(0)+sollSpeedGelenk.get(0)*deltaT);
			gelenkNeu.add(qt.get(1)+sollSpeedGelenk.get(1)*deltaT);
			gelenkNeu.add(qt.get(2)+sollSpeedGelenk.get(2)*deltaT);
			gelenkNeu.add(qt.get(3)+sollSpeedGelenk.get(3)*deltaT);
			gelenkNeu.add(qt.get(4)+sollSpeedGelenk.get(4)*deltaT);
			
			//aus qt wt berechnen
			qtTOwt = r.getToolConfigVektor(gelenkNeu);
			System.out.println("Tool Config berechnet:\n"+qtTOwt);
			
			
			wLast = (Vector<Double>) wNeu.clone();
			System.out.println();
		}
		

	}

}
