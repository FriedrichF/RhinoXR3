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
		Vector<Double> qt = r.getGelenkvariablenVektor(startPunkt);
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
			System.out.println("neues w:\n"+wNeu);		//Passt!!!!!!!
			
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
			sollSpeedGelenk = r.getGeschwindigkeitGelenkvariabel(wLast, sollSpeedTool, qt);
			System.out.println("Sollgeschwindigkeit Gelenk:\n"+sollSpeedGelenk);
			
			//Neue Position im Gelenkraum simulieren
			qt.set(0,qt.get(0)+sollSpeedGelenk.get(0)*deltaT);
			qt.set(1,qt.get(1)+sollSpeedGelenk.get(1)*deltaT);
			qt.set(2,qt.get(2)+sollSpeedGelenk.get(2)*deltaT);
			qt.set(3,qt.get(3)+sollSpeedGelenk.get(3)*deltaT);
			qt.set(4,qt.get(4)+sollSpeedGelenk.get(4)*deltaT);
			System.out.println("Gelenkraum berechnet:\n"+r.getGelenkvariablenVektor(wNeu));
			System.out.println("Gelenkraum Neu:\n"+qt);
			
			
			//aus qt wt berechnen
			qtTOwt = r.getToolConfigVektor(qt);
			System.out.println("Tool Config berechnet:\n"+qtTOwt);
			
			
			wLast = (Vector<Double>) wNeu.clone();
			System.out.println();
		}
		
		System.out.println(vectorToString(qtTOwt));
		

	}
	
	public static String vectorToString(Vector<Double> v){
		
		String s="";
		
		for(Double d: v){
			s+=runde(d)+"\r";
		}
		
		return s+"\r";
		
	}
	
	/**
	 * 
	 * Rundet auf 6 Nachkommastellen
	 * 
	 * @param wert der gerundet werden soll
	 * @return den gerundeten Wert zurück
	 */
	private static Double runde(Double wert){
		return (double) Math.round(wert * 10000.0) / 10000.0;
		
	}

}
