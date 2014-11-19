package robotik7;

import java.util.Vector;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.atan2;
import static java.lang.Math.acos;
import static java.lang.Math.PI;
import static java.lang.Math.E;
import static java.lang.Math.log;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class RhinoXR3Kinematic {
	//Der überschaubarkeit zu gute, wurde weniger kaskadiert und einige deklarationen mehrfach ausgeführt
	
	
	private final Double a2 = 0.2286;
	private final Double a3 = 0.2286;
	private final Double a4 = 0.95;
	
	private final Double d1 = 0.2604;
	private final Double d5 = 0.1683;
	
	private Double q234;
	private Double b0;
	private Double b1;
	private Double b2;
	private Double bt1, bt2;
	private Double qt234;
	
	

	
	public Vector<Double> getGeschwindigkeitGelenkvariabel(Vector<Double> w, Vector<Double> wt, Vector<Double> q){
		Double wt1	= w.get(0);
		Double wt2	= w.get(1);
		Double wt3	= w.get(2);
		Double wt4	= w.get(3);
		Double wt5	= w.get(4);
		Double wt6	= w.get(5);
		
		Double q1	= q.get(0);
		Double q2	= q.get(1);
		Double q3	= q.get(2);
		Double q4 	= q.get(3);
		
		Double w1	= w.get(0);
		Double w2	= w.get(1);
		Double w3	= w.get(2);
		Double w4	= w.get(3);
		Double w5	= w.get(4);
		Double w6	= w.get(5);
		
		b0 			= cos(q1)*w4+sin(q1)*w5;
		q234		= Math.atan2(-b0, -w6);
		b1			= cos(q1)*w1+sin(q1)*w2-a4*cos(q234)+d5*sin(q234);
		b2			= d1 - a4*sin(q234) - d5*cos(q234)-w3;
		
		
		//t für Ableitung nach zeit
		Double qt1, qt2, qt3, qt4, qt5;
		qt1 = bestimme_qt1(w1, wt2, w2, wt1);
		qt3 = bestimme_qt3(w, wt, q, qt1);
		qt2 = bestimme_qt2(w, wt, q, qt1, qt3);
		qt4	= bestimme_qt4(qt2,qt3);
		qt5 = bestimme_qt5(w,wt);
		
		Vector<Double> qt = new Vector<Double>();
		qt.add(qt1);
		qt.add(qt2);
		qt.add(qt3);
		qt.add(qt4);
		qt.add(qt5);
		
		return qt;
	}
	
	
	private Double bestimme_qt1(Double w1, Double wt2, Double w2, Double wt1 ){
		
		//skript 4 seite 51, Geschwindigkeit basis
		return (w1*wt2-w2*wt1)/(w1*w1+w2*w2);		
	}
	
	
	private Double bestimme_qt2(Vector<Double> w, Vector<Double> wt, Vector<Double> q, Double qt1, Double qt3 ){
		
		Double wt1	= w.get(0);
		Double wt2	= w.get(1);
		Double wt3	= w.get(2);
		Double wt4	= w.get(3);
		Double wt5	= w.get(4);
		Double wt6	= w.get(5);
		
		Double w1	= w.get(0);
		Double w2	= w.get(1);
		Double w3	= w.get(2);
		Double w4	= w.get(3);
		Double w5	= w.get(4);
		Double w6	= w.get(5);
		
		Double q1	= q.get(0);
		Double q2	= q.get(1);
		Double q3	= q.get(2);
		Double q4 	= q.get(3);		
		
		Double b3 	= (a2+a3*cos(q3)*b1+a3*sin(q3)*b2);
		Double b4 	= (a2+a3*cos(q3)*b2+a3*sin(q3)*b1);
		Double bt3 	= (a2+a3*cos(q3)*bt1+a3*sin(q3)*bt2	+ a3*(cos(q3)*b2-sin(q3)*b1)*qt3);
		Double bt4 	= (a2+a3*cos(q3)*bt2+a3*sin(q3)*bt1	- a3*(cos(q3)*b1+sin(q3)*b2)*qt3);
		
		Double qt2	= (b3*bt4-b4*bt3)/(b3*b3+b4*b4);
		
		return qt2;
	}
	
	/**
	 * Hat zwei Lösungen!!!! qt3 und -qt3
	 * 
	 * @param w Tool-Konfigurationsvektor
	 * @param wt Geschwindigkeit im Tool-Konfigurationsraum als Vektor
	 * @param q Vektor der Gelenkvariablen
	 * @param qt1 Geschwindigkeit im Gelenkraum q1
	 * @return Geschwindigkeit im Gelenkraum q3
	 */
	private Double bestimme_qt3(Vector<Double> w, Vector<Double> wt, Vector<Double> q, Double qt1){
		
		Double wt1	= w.get(0);
		Double wt2	= w.get(1);
		Double wt3	= w.get(2);
		Double wt4	= w.get(3);
		Double wt5	= w.get(4);
		Double wt6	= w.get(5);
		
		Double w1	= w.get(0);
		Double w2	= w.get(1);
		Double w4	= w.get(3);
		Double w5	= w.get(4);
		Double w6	= w.get(5);
		
		Double q1	= q.get(0);
		Double q2	= q.get(1);
		Double q3	= q.get(2);
		Double q23	= q2+q3;		
		
		//skript 4, Seite 52 Geschwindigkeit Ellbogengelenk
		Double bt0	= cos(q1)*wt4+sin(q1)*wt5+(-sin(q1)*w4+cos(q1)*w5)*qt1;
		qt234 = (w6*bt0-b0*wt6)/(w6*w6+b0*b0);
		bt1	= cos(q1)*wt1+sin(q1)*wt2+(cos(q1)*w2-sin(q1)*w1)*qt1+(a4*sin(q234)+d5*cos(q234)*qt234);
		bt2	= (d5*sin(q234)-a4*cos(q234))*qt234-wt3;	
		
		Double x	= ((b1*b1)+(b2*b2)-(a2*a2)-(a3*a3));
		Double y	= (2*a2*a3);
		Double z	= 2*(b1*bt1+b2*bt2);
		
		Double qt3	= z / sqrt(y*y-x*x);
		
		return qt3;
	}

	private Double bestimme_qt4(Double qt2, Double qt3){
		
		return qt234-qt2-qt3;
		
	}
	
	private Double bestimme_qt5(Vector<Double> w, Vector<Double> wt){
		
		Double wt4	= w.get(3);
		Double wt5	= w.get(4);
		Double wt6	= w.get(5);
		
		Double w4	= w.get(3);
		Double w5	= w.get(4);
		Double w6	= w.get(5);
		
		Double qt5 	= (PI*(w4*wt4+w5*wt5+w6*wt6))/(w4*w4+w5*w5+w6*w6);
		
		return qt5;
	}
	
	public Vector<Double> getToolConfigVektor(Vector<Double> gelenkvariablen_vektor){
		
		if(gelenkvariablen_vektor.size() != 5){
			throw new IllegalArgumentException("gelenkvariablen_vektor hat die falsche länge!!!");
		}
		
		Double q1, q2, q3, q4, q5;
		q1	= gelenkvariablen_vektor.get(0);
		q2	= gelenkvariablen_vektor.get(1);		
		q3	= gelenkvariablen_vektor.get(2);
		q4	= gelenkvariablen_vektor.get(3);
		q5	= gelenkvariablen_vektor.get(4);
		
		Double w1,w2,w3,w4,w5,w6;
		w1	= bestimme_w1(q1, q2, q3, q4);
		w2	= bestimme_w2(q1, q2, q3, q4);		
		w3	= bestimme_w3(q1, q2, q3, q4);		
		w4	= bestimme_w4(q1, q2, q3, q4, q5);
		w5	= bestimme_w5(q1, q2, q3, q4, q5);
		w6	= bestimme_w6(q1, q2, q3, q4, q5);
		
		Vector<Double> w = new Vector<Double>();
		w.add(w1);
		w.add(w2);
		w.add(w3);
		w.add(w4);
		w.add(w5);
		w.add(w6);
		
		return w;
	}
	
	
	
	private Double bestimme_w1(Double q1, Double q2, Double q3, Double q4){

		q234		= q2+q3+q4;
		Double q23	= q2+q3;
		
		//w1 = C1(a2C2+a3C23+a4C234-d5S234)
		Double w1 	= cos(q1)*(a2*cos(q2)+a3*cos(q23)+a4*cos(q234)-d5*sin(q234));
		
		return w1;
		
	}
	
	private Double bestimme_w2(Double q1, Double q2, Double q3, Double q4){

		q234		= runde(q2+q3+q4);
		Double q23	= runde(q2+q3);
		
		//w2 = S1(a2C2+a3C23+a4C234-d5S234)
		Double w2 	= sin(q1)*(a2*cos(q2)+a3*cos(q23)+a4*cos(q234)-d5*sin(q234));
		return w2;
		
	}
	
	private Double bestimme_w3(Double q1, Double q2, Double q3, Double q4){

		q234		= q2+q3+q4;
		Double q23	= q2+q3;
		
		//w3 = d1-a2S-a3S2-a4S234-d5C234
		Double w3 	= d1	-a2*sin(q2)	-a3*sin(q23) 	-a4*sin(q234)	-d5*cos(q234);
		
		return w3;
		
	}
	
	private Double bestimme_w4(Double q1, Double q2, Double q3, Double q4, Double q5){

		q234		= q2+q3+q4;
		
		//w4 = -exp(q5/pi)C1*S234
		
		Double w4 	= -pow(E, (q5/PI))*cos(q1)*sin(q234);
		
		return w4;
		
	}
	
	private Double bestimme_w5(Double q1, Double q2, Double q3, Double q4, Double q5){

		q234		= q2+q3+q4;
		
		//w4 = -exp(q5/pi)S1*S234
		
		Double w5 	= -pow(E, (q5/PI))*sin(q1)*sin(q234);
		
		return w5;
		
	}
	
	private Double bestimme_w6(Double q1, Double q2, Double q3, Double q4, Double q5){

		q234		= q2+q3+q4;
		
		//w4 = -exp(q5/pi)C234
		
		Double w6 	= -pow(E, (q5/PI))*cos(q234);
		
		return w6;
		
	}
	
	public Vector<Double> getGelenkvariablenVektor(Vector<Double> tool_config_vektor){
		
		Vector<Double> w = tool_config_vektor;
		
		if(w.size() != 6){
			throw new IllegalArgumentException("Tool config Vektor hat die falsche länge!!!");
		}
		
		Double q1,q2,q3,q4,q5;
		Double w1, w2;
		
		w1	= w.elementAt(0);	
		w2	= w.elementAt(1);
		
		q1 	= bestimme_q1(w1, w2);
		q3	= bestimme_q3(w, q1);
		q2	= bestimme_q2(q3);
		q4	= bestimme_q4(q2, q3);
		q5	= bestimme_q5(tool_config_vektor);
		
		Vector<Double> gelenkvariablen_vektor = new Vector<Double>();
		gelenkvariablen_vektor.add(q1);
		gelenkvariablen_vektor.add(q2);
		gelenkvariablen_vektor.add(q3);
		gelenkvariablen_vektor.add(q4);
		gelenkvariablen_vektor.add(q5);

		
		
		return gelenkvariablen_vektor;
	}
	
	private Double bestimme_q1(Double w1, Double w2){
		
		Double q1 = atan2(w2, w1);
		
		return q1;
	}
	
	private Double bestimme_q3(Vector<Double> tool_config_vektor, Double q1){

		//nach gerenchnet keinen Fehler !!!gefunden!!!
		
		Double w1	= tool_config_vektor.get(0);
		Double w2	= tool_config_vektor.get(1);
		Double w3	= tool_config_vektor.get(2);
		Double w4	= tool_config_vektor.get(3);
		Double w5	= tool_config_vektor.get(4);
		Double w6	= tool_config_vektor.get(5);
		
		Double b0	= cos(q1)*w4	+sin(q1)*w5;

		this.q234 	= atan2(-b0,	-w6);
		this.b1		= cos(q1)*w1	+sin(q1)*w2		-a4*cos(q234)	+ d5*sin(q234);
		this.b2		= d1	-a4*sin(q234)	-d5*cos(q234)	-w3;
		double bogen = Math.toRadians((pow(b1,2)+pow(b2,2)-pow(a2,2)-pow(a3,2))/(2*a2*a3));

		double q3	= acos(bogen);
		
		return q3;
	}
	
	private Double bestimme_q2(Double q3){
		
		Double y 	= (a2+a3*cos(q3))*b2-a3*sin(q3)*b1;
		Double x 	= (a2+a3*cos(q3))*b1+a3*sin(q3)*b2;
	
		Double q2 	= atan2(y, x);
		
		return q2;
	}
	
	private Double bestimme_q4(Double q2, Double q3){
		
		Double q4 = q234-q2-q3;

		return q4;
	}
	
	private Double bestimme_q5(Vector<Double> tool_config_vektor){
		
		Double w4 = tool_config_vektor.get(3);
		Double w5 = tool_config_vektor.get(4);
		Double w6 = tool_config_vektor.get(5);
		
		Double q5 = PI*log(sqrt(w4*w4+w5*w5+w6*w6));
		
		return q5;
	}
	
	public Double speed(Double t, Double T, Double tau){
		if(t >= T)
			return 1.0;
		else if(t == 0)
			return 0.0;
		else if(t<tau)
			return 0.5/(tau*(T-tau))*t*t;
		else if(t>(T-tau))
			return (-0.5*t*t+T*t)/(tau*(T-tau))+1+((-0.5*T*T)/((T-tau)*tau));
		else
			return t/(T-tau)-0.5*(tau/(T-tau));
	}
	
	public String vectorToString(Vector<Double> v){
		
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
	private Double runde(Double wert){
		return (double) Math.round(wert * 1000000.0) / 1000000.0;
		
	}
	
}
